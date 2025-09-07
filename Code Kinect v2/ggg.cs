using System;
using System.Windows;
using Microsoft.Kinect;
using System.Windows.Media.Imaging;
using System.Windows.Media;
using System.Runtime.InteropServices;

namespace KinectFloorHighlighter
{
    public partial class MainWindow : Window
    {
        KinectSensor kinect;
        CoordinateMapper coordinateMapper;
        MultiSourceFrameReader reader;

        byte[] colorPixels;
        WriteableBitmap bitmap;

        FloorClipPlane floor;

        public MainWindow()
        {
            InitializeComponent();

            kinect = KinectSensor.GetDefault();
            kinect.Open();

            coordinateMapper = kinect.CoordinateMapper;
            reader = kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Body);

            var frameDesc = kinect.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            colorPixels = new byte[frameDesc.Width * frameDesc.Height * 4];
            bitmap = new WriteableBitmap(frameDesc.Width, frameDesc.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            CameraImage.Source = bitmap;

            reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            using (var colorFrame = reference.ColorFrameReference.AcquireFrame())
            using (var depthFrame = reference.DepthFrameReference.AcquireFrame())
            using (var bodyFrame = reference.BodyFrameReference.AcquireFrame())
            {
                if (colorFrame == null || depthFrame == null || bodyFrame == null) return;

                // Cập nhật mặt phẳng sàn
                foreach (var body in bodyFrame.Bodies)
                {
                    if (body != null && body.IsTracked)
                    {
                        floor = bodyFrame.FloorClipPlane;
                        break;
                    }
                }

                // Lấy dữ liệu ảnh màu
                colorFrame.CopyConvertedFrameDataToArray(colorPixels, ColorImageFormat.Bgra);

                // Dữ liệu độ sâu
                ushort[] depthData = new ushort[depthFrame.FrameDescription.LengthInPixels];
                depthFrame.CopyFrameDataToArray(depthData);

                // Ánh xạ
                CameraSpacePoint[] cameraPoints = new CameraSpacePoint[depthData.Length];
                coordinateMapper.MapDepthFrameToCameraSpace(depthData, cameraPoints);

                // Xử lý: đánh dấu điểm gần sàn
                for (int i = 0; i < cameraPoints.Length; i++)
                {
                    CameraSpacePoint point = cameraPoints[i];
                    if (float.IsInfinity(point.X) || float.IsInfinity(point.Y) || float.IsInfinity(point.Z))
                        continue;

                    // Tính khoảng cách điểm đến mặt phẳng sàn
                    float d = floor.X * point.X + floor.Y * point.Y + floor.Z * point.Z + floor.W;

                    // Nếu điểm gần sàn (±10 cm)
                    if (Math.Abs(d) < 0.1f)
                    {
                        int colorIndex = i * 4;
                        if (colorIndex + 3 < colorPixels.Length)
                        {
                            colorPixels[colorIndex + 0] = 0;    // B
                            colorPixels[colorIndex + 1] = 0;    // G
                            colorPixels[colorIndex + 2] = 255;  // R
                        }
                    }
                }

                // Hiển thị ảnh
                bitmap.WritePixels(
                    new Int32Rect(0, 0, bitmap.PixelWidth, bitmap.PixelHeight),
                    colorPixels, bitmap.PixelWidth * 4, 0);
            }
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (reader != null) reader.Dispose();
            if (kinect != null) kinect.Close();
        }
    }
}
