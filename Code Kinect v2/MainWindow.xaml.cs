using Microsoft.Kinect;
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace KinectRedMarker
{
    public partial class MainWindow : Window
    {
        KinectSensor sensor;
        MultiSourceFrameReader reader;
        CoordinateMapper coordinateMapper;
        BodyFrameReader bodyReader;
        Body[] bodies;

        byte[] colorData;
        ushort[] depthData;
        DepthSpacePoint[] depthSpacePoints;
        WriteableBitmap bitmap;
        DrawingGroup drawingGroup;

        CameraSpacePoint? redPt3D = null;
        CameraSpacePoint? greenPt3D = null; // ✅ marker ảo
        CameraSpacePoint rightWrist, rightElbow, spineShoulder, spineMid;

        UdpClient udpClient;
        IPEndPoint pythonEndPoint;
        DateTime lastSent = DateTime.MinValue;

        public MainWindow()
        {
            InitializeComponent();

            sensor = KinectSensor.GetDefault();
            if (sensor != null)
            {
                sensor.Open();
                coordinateMapper = sensor.CoordinateMapper;
                reader = sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth);
                reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                bodyReader = sensor.BodyFrameSource.OpenReader();
                bodyReader.FrameArrived += BodyReader_FrameArrived;
                bodies = new Body[sensor.BodyFrameSource.BodyCount];
            }

            drawingGroup = new DrawingGroup();
            cameraView.Source = new DrawingImage(drawingGroup);

            // UDP init
            udpClient = new UdpClient();
            pythonEndPoint = new IPEndPoint(IPAddress.Loopback, 5005);
        }

        private void BodyReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame == null) return;
                frame.GetAndRefreshBodyData(bodies);
                foreach (var body in bodies)
                {
                    if (!body.IsTracked) continue;
                    rightWrist = body.Joints[JointType.WristRight].Position;
                    rightElbow = body.Joints[JointType.ElbowRight].Position;
                    spineShoulder = body.Joints[JointType.SpineShoulder].Position;
                    spineMid = body.Joints[JointType.SpineMid].Position;

                    // ✅ marker ảo: ngang hông (spineMid) + dịch sang phải 0.5m
                    greenPt3D = new CameraSpacePoint
                    {
                        X = spineMid.X + 0.5f,
                        Y = spineMid.Y,
                        Z = spineMid.Z
                    };
                }
            }
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var frame = e.FrameReference.AcquireFrame();
            if (frame == null) return;

            ColorFrame colorFrame = null;
            DepthFrame depthFrame = null;

            try
            {
                colorFrame = frame.ColorFrameReference.AcquireFrame();
                depthFrame = frame.DepthFrameReference.AcquireFrame();
                if (colorFrame == null || depthFrame == null) return;

                int colorWidth = colorFrame.FrameDescription.Width;
                int colorHeight = colorFrame.FrameDescription.Height;
                int depthWidth = depthFrame.FrameDescription.Width;
                int depthHeight = depthFrame.FrameDescription.Height;

                if (colorData == null) colorData = new byte[colorWidth * colorHeight * 4];
                if (depthData == null) depthData = new ushort[depthWidth * depthHeight];
                if (depthSpacePoints == null) depthSpacePoints = new DepthSpacePoint[colorWidth * colorHeight];

                colorFrame.CopyConvertedFrameDataToArray(colorData, ColorImageFormat.Bgra);
                depthFrame.CopyFrameDataToArray(depthData);

                if (bitmap == null)
                    bitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

                bitmap.WritePixels(new Int32Rect(0, 0, colorWidth, colorHeight), colorData, colorWidth * 4, 0);

                // --- Tìm marker đỏ ---
                var redPt2D = FindMarker(colorData, colorWidth, colorHeight, Colors.Red, 75);
                if (redPt2D.HasValue)
                {
                    coordinateMapper.MapColorFrameToDepthSpace(depthData, depthSpacePoints);
                    redPt3D = MapTo3D(redPt2D.Value, depthData, depthSpacePoints, coordinateMapper, colorWidth);
                }
                else redPt3D = null;

                using (var dc = drawingGroup.Open())
                {
                    dc.DrawImage(bitmap, new Rect(0, 0, bitmap.PixelWidth, bitmap.PixelHeight));

                    if (redPt2D.HasValue) DrawJoint(dc, redPt2D.Value, Brushes.Red);
                    DrawJoint2D(dc, rightWrist, colorWidth, colorHeight, Brushes.Blue);
                    DrawJoint2D(dc, rightElbow, colorWidth, colorHeight, Brushes.Green);
                    DrawBone2D(dc, rightElbow, rightWrist, colorWidth, colorHeight, Brushes.Yellow, 3);

                    // ✅ Vẽ marker ảo (màu xanh lá)
                    if (greenPt3D.HasValue)
                    {
                        var colorPt = coordinateMapper.MapCameraPointToColorSpace(greenPt3D.Value);
                        if (!double.IsNaN(colorPt.X) && !double.IsNaN(colorPt.Y))
                        {
                            dc.DrawEllipse(Brushes.Lime, new Pen(Brushes.Black, 2),
                                new System.Windows.Point(colorPt.X, colorPt.Y), 15, 15);
                        }
                    }
                }

                // --- Hiển thị khoảng cách ---
                if (redPt3D.HasValue && spineShoulder.Z > 0)
                {
                    double dx = redPt3D.Value.X - spineShoulder.X;
                    double dy = redPt3D.Value.Y - spineShoulder.Y;
                    double dz = redPt3D.Value.Z - spineShoulder.Z;
                    double dist = Math.Sqrt(dx * dx + dy * dy + dz * dz);

                    if (dist <= 0.45) // 45 cm
                        DistanceText.Text = $"⚠ Nguy hiểm! d={dist:F2} m";
                    else if (dist <= 1.5) // 150 cm
                        DistanceText.Text = $"🤝 Bắt đầu handover (robot chậm lại), d={dist:F2} m";
                    else
                        DistanceText.Text = $"✅ An toàn, d={dist:F2} m";
                }
                else
                {
                    DistanceText.Text = "⏳ Chưa phát hiện body hoặc marker";
                }

                // --- Thông tin marker đỏ ---
                if (redPt3D.HasValue)
                {
                    RedMarkerText.Text = $"Marker ĐỎ: X={redPt3D.Value.X:F2}, Y={redPt3D.Value.Y:F2}, Z={redPt3D.Value.Z:F2}";
                }
                else
                {
                    RedMarkerText.Text = "Red Marker not found";
                }

                // --- Thông tin marker ảo (xanh) ---
                if (greenPt3D.HasValue)
                {
                    GreenMarkerText.Text = $"Marker XANH (ảo): X={greenPt3D.Value.X:F2}, Y={greenPt3D.Value.Y:F2}, Z={greenPt3D.Value.Z:F2}";
                }
                else
                {
                    GreenMarkerText.Text = "Green Marker not found";
                }

                // --- Gửi dữ liệu qua UDP 20Hz ---
                // --- Gửi dữ liệu qua UDP 20Hz ---
                if ((DateTime.Now - lastSent).TotalMilliseconds >= 50)
                {
                    double distToRed = 0.0;
                    if (redPt3D.HasValue && spineShoulder.Z > 0)
                    {
                        double dx = redPt3D.Value.X - spineShoulder.X;
                        double dy = redPt3D.Value.Y - spineShoulder.Y;
                        double dz = redPt3D.Value.Z - spineShoulder.Z;
                        distToRed = Math.Sqrt(dx * dx + dy * dy + dz * dz);
                    }

                    string msg = $"{distToRed:F3}," +   // ✅ thay 3 số XYZ marker đỏ bằng 1 số khoảng cách
                                 $"{rightWrist.X},{rightWrist.Y},{rightWrist.Z}," +
                                 $"{rightElbow.X},{rightElbow.Y},{rightElbow.Z}," +
                                 $"{(greenPt3D?.X ?? 0)},{(greenPt3D?.Y ?? 0)},{(greenPt3D?.Z ?? 0)}";
                    byte[] data = Encoding.UTF8.GetBytes(msg);
                    udpClient.Send(data, data.Length, pythonEndPoint);
                    lastSent = DateTime.Now;
                }

            }
            finally
            {
                colorFrame?.Dispose();
                depthFrame?.Dispose();
            }
        }

        // --- Vẽ 2D joints ---
        private void DrawJoint(DrawingContext dc, System.Windows.Point p, Brush brush)
        {
            if (double.IsNaN(p.X) || double.IsNaN(p.Y)) return;
            dc.DrawEllipse(brush, new Pen(Brushes.White, 2), p, 15, 15);
        }

        private void DrawJoint2D(DrawingContext dc, CameraSpacePoint joint, int imgWidth, int imgHeight, Brush brush)
        {
            if (joint.Z <= 0) return;
            var colorPt = coordinateMapper.MapCameraPointToColorSpace(joint);
            if (double.IsNaN(colorPt.X) || double.IsNaN(colorPt.Y)) return;
            dc.DrawEllipse(brush, new Pen(Brushes.White, 2), new System.Windows.Point(colorPt.X, colorPt.Y), 10, 10);
        }

        private void DrawBone2D(DrawingContext dc, CameraSpacePoint joint1, CameraSpacePoint joint2, int imgWidth, int imgHeight, Brush brush, double thickness)
        {
            if (joint1.Z <= 0 || joint2.Z <= 0) return;
            var pt1 = coordinateMapper.MapCameraPointToColorSpace(joint1);
            var pt2 = coordinateMapper.MapCameraPointToColorSpace(joint2);
            if (double.IsNaN(pt1.X) || double.IsNaN(pt1.Y) || double.IsNaN(pt2.X) || double.IsNaN(pt2.Y)) return;
            dc.DrawLine(new Pen(brush, thickness), new System.Windows.Point(pt1.X, pt1.Y), new System.Windows.Point(pt2.X, pt2.Y));
        }

        private System.Windows.Point? FindMarker(byte[] data, int width, int height, Color target, int tolerance)
        {
            int sumX = 0, sumY = 0, count = 0;
            for (int y = 0; y < height; y += 2)
            {
                for (int x = 0; x < width; x += 2)
                {
                    int idx = (y * width + x) * 4;
                    byte b = data[idx], g = data[idx + 1], r = data[idx + 2];
                    if (Math.Abs(r - target.R) < tolerance &&
                        Math.Abs(g - target.G) < tolerance &&
                        Math.Abs(b - target.B) < tolerance)
                    {
                        sumX += x;
                        sumY += y;
                        count++;
                    }
                }
            }
            if (count == 0) return null;
            var pt = new System.Windows.Point(sumX / count, sumY / count);
            if (double.IsNaN(pt.X) || double.IsNaN(pt.Y)) return null;
            return pt;
        }

        private CameraSpacePoint? MapTo3D(System.Windows.Point colorPt, ushort[] depthData, DepthSpacePoint[] depthSpacePoints, CoordinateMapper mapper, int colorWidth)
        {
            int colorIndex = (int)colorPt.Y * colorWidth + (int)colorPt.X;
            if (colorIndex < 0 || colorIndex >= depthSpacePoints.Length) return null;

            var depthPt = depthSpacePoints[colorIndex];
            if (double.IsNaN(depthPt.X) || double.IsNaN(depthPt.Y)) return null;

            int dx = (int)Math.Round(depthPt.X);
            int dy = (int)Math.Round(depthPt.Y);
            if (dx >= 0 && dx < 512 && dy >= 0 && dy < 424)
            {
                int depthIndex = dy * 512 + dx;
                ushort depth = depthData[depthIndex];
                if (depth > 0)
                    return mapper.MapDepthPointToCameraSpace(depthPt, depth);
            }
            return null;
        }

        protected override void OnClosed(EventArgs e)
        {
            reader?.Dispose();
            bodyReader?.Dispose();
            sensor?.Close();
            base.OnClosed(e);
        }
    }
}
