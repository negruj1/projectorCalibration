//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.CoordinateMappingBasics
{
    using System;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Linq;
    using Microsoft.Kinect;
    using System.Windows.Forms;
    using System.Collections.Generic;
    using System.Windows.Media.Media3D;
    //using System.Timers;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {


        /// <summary>
        /// Size of the RGB pixel in the bitmap
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for depth/color/body index frames
        /// </summary>
        private MultiSourceFrameReader multiFrameSourceReader = null;

        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap bitmap = null;

        private WriteableBitmap firstImage = null;

        private Boolean writeImageToFirstImageBuffer = false;
        private Boolean projectingStarted = false;
        private Boolean verticalProjectingDone = false;
        private Boolean horizontalProjectingDone = false;
        private Boolean depthFrameFiltered = false;
        private Boolean runningCalibration = false;
        private Boolean depthScreenshotTaken = false;
        private Boolean calculatingStarted = false;
        private int frameCounter = 0;
        private int projectionImageCounter = 0;
        private List<ProjectablePattern> horizontalImages= null;
        private List<ProjectablePattern> verticalImages = null;

        private Boolean captureDepthFrame = false;
        private Boolean depthFrameCaptured = false;
        private ushort minDepthRange = 0;
        private ushort maxDepthRange = 0;

        ushort[] globalDepth = new ushort[512 * 424];
        Boolean[] globalDepthFilteredLookup = new Boolean[512 * 424];

        /// <summary>
        /// The size in bytes of the bitmap back buffer
        /// </summary>
        private uint bitmapBackBufferSize = 0;


        private int projectorWidth = 1280;
        private int projectorHeight = 800;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private DepthSpacePoint[] colorMappedToDepthPoints = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        private Microsoft.Samples.Kinect.CoordinateMappingBasics.Projector myProjector = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            this.kinectSensor = KinectSensor.GetDefault();

            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Depth | FrameSourceTypes.Color | FrameSourceTypes.BodyIndex);

            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            FrameDescription depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            int depthWidth = depthFrameDescription.Width;
            int depthHeight = depthFrameDescription.Height;

            Console.WriteLine("DepthFrame dimensions: {0}, {1}", depthWidth, depthHeight);
            

            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            int colorWidth = colorFrameDescription.Width;
            int colorHeight = colorFrameDescription.Height;

            Console.WriteLine("ColorFrame dimensions: {0}, {1}", colorWidth, colorHeight);

            this.colorMappedToDepthPoints = new DepthSpacePoint[colorWidth * colorHeight];

            this.bitmap = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);
            this.firstImage = new WriteableBitmap(colorWidth, colorHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

            Console.WriteLine("Bitmap dimensions: {0}, {1}", this.bitmap.Width, this.bitmap.Height);

            
            // Calculate the WriteableBitmap back buffer size
            this.bitmapBackBufferSize = (uint)((this.bitmap.BackBufferStride * (this.bitmap.PixelHeight - 1)) + (this.bitmap.PixelWidth * this.bytesPerPixel));
                                   
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            this.kinectSensor.Open();

            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            this.DataContext = this;

            var screen1 = System.Windows.Forms.Screen.AllScreens[0].Bounds;
            var screen2 = System.Windows.Forms.Screen.AllScreens[1].WorkingArea;
            //var bound = System.Windows.Forms.Screen.GetWorkingArea(new Control());


            Debug.Assert(System.Windows.Forms.SystemInformation.MonitorCount > 1);

            this.horizontalImages = new List<ProjectablePattern>();
            this.verticalImages = new List<ProjectablePattern>();

            var projector = new Projector();

            double width = SystemParameters.VirtualScreenWidth;
            double height = SystemParameters.VirtualScreenHeight;
            Console.WriteLine("screen1: {0}, {1}   screen2: {2}, {3}", screen1.ToString(), screen1.Height, screen2.ToString(), screen2.Height);
            
            projector.Left = 1920;
            projector.Top = 0;
            projector.Width = this.projectorWidth;
            projector.Height = this.projectorHeight;
            projector.WindowStyle = WindowStyle.None;
            projector.AllowsTransparency = true;
            
            projector.Show();
            //projector.WindowState = WindowState.Maximized;
            
            

            this.myProjector = projector;

            this.myProjector.changeCanvasSize((int)projector.Width, (int)projector.Height);



            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.bitmap;
            }
        }


        /*public unsafe static BitmapSource ToGrayScale(BitmapSource source)
        {
            const int PIXEL_SIZE = 4;
            int width = source.PixelWidth;
            int height = source.PixelHeight;
            var bitmap = new WriteableBitmap(source);

            bitmap.Lock();
            var backBuffer = (byte*)bitmap.BackBuffer.ToPointer();
            for (int y = 0; y < height; y++)
            {
                var row = backBuffer + (y * bitmap.BackBufferStride);
                for (int x = 0; x < width; x++)
                {
                    var grayScale = (byte)(((row[x * PIXEL_SIZE + 1]) + (row[x * PIXEL_SIZE + 2]) + (row[x * PIXEL_SIZE + 3])) / 3);
                    for (int i = 0; i < PIXEL_SIZE; i++)
                        row[x * PIXEL_SIZE + i] = grayScale;
                }
            }
            bitmap.AddDirtyRect(new Int32Rect(0, 0, width, height));
            bitmap.Unlock();

            return bitmap;
        }*/


        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.multiFrameSourceReader != null)
            {
                // MultiSourceFrameReader is IDisposable
                this.multiFrameSourceReader.Dispose();
                this.multiFrameSourceReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }



        private void projectButton_Click(object sender, RoutedEventArgs e) {
            Console.WriteLine("projectButton_Click");
            this.writeImageToFirstImageBuffer = true;
           
        }

        private void filterDepth2(int surroundingPixels) {
            if (this.depthFrameCaptured)
            {
                int depthWidth = 512;
                int depthHeight = 424;
                ushort threshold = 15;

                for (int y = 0; y < depthHeight; y++)
                {
                    for (int x = 0; x < depthWidth; x++)
                    {

                        ushort currentDepth = this.globalDepth[y * depthWidth + x];

                        if (y >= surroundingPixels && y < depthHeight - surroundingPixels && x >= surroundingPixels && x < depthWidth - surroundingPixels)
                        {
                            Boolean isOk = true;

                            for (int innerY = (-1) * surroundingPixels; innerY < surroundingPixels;innerY++)
                            {
                                for (int innerX = (-1) * surroundingPixels; innerX < surroundingPixels; innerX++)
                                {
                                    if(!isOk){
                                        continue;
                                    }
                                    int thisThreshold = Math.Max(Math.Abs(innerY), Math.Abs(innerX))*threshold;
                                    if (Math.Abs(this.globalDepth[(y + innerY) * depthWidth + x + innerX] - currentDepth) > thisThreshold)
                                    {
                                        isOk = false;
                                        continue;
                                    }
                                }
                            }

                            if (isOk)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = true;
                            }
                            else {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                            }
                            
                        }
                        else
                        {
                            this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                        }
                    }
                }
                this.depthFrameFiltered = true;
            }
            else
            {
                Console.WriteLine("Not filtered yet!!");
            }
        }

        private void filterDepth() {
            if (this.depthFrameCaptured)
            {
                int depthWidth = 512;
                int depthHeight = 424;
                ushort threshold = 10;
                ushort outerThreshold = 20;

                for (int y = 0; y < depthHeight; y++)
                {
                    for (int x = 0; x < depthWidth; x++)
                    {

                        ushort currentDepth = this.globalDepth[y * depthWidth + x];

                        if (y > 1 && y < depthHeight - 2 && x > 1 && x < depthWidth - 2)
                        {
                            Boolean tooFarAway = Math.Abs(currentDepth) > 5000;
                            if (tooFarAway)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelAboveLeft = Math.Abs(this.globalDepth[(y - 1) * depthWidth + x - 1] - currentDepth) < threshold;
                            if (!pixelAboveLeft)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelAbove = Math.Abs(this.globalDepth[(y - 1) * depthWidth + x] - currentDepth) < threshold;
                            if (!pixelAbove)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelAboveRight = Math.Abs(this.globalDepth[(y - 1) * depthWidth + x + 1] - currentDepth) < threshold;
                            if (!pixelAboveRight)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelLeft = Math.Abs(this.globalDepth[y * depthWidth + x - 1] - currentDepth) < threshold;
                            if (!pixelLeft)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelRight = Math.Abs(this.globalDepth[y * depthWidth + x - 1] - currentDepth) < threshold;
                            if (!pixelRight)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelBelowLeft = Math.Abs(this.globalDepth[(y + 1) * depthWidth + x - 1] - currentDepth) < threshold;
                            if (!pixelBelowLeft)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelBelow = Math.Abs(this.globalDepth[(y + 1) * depthWidth + x] - currentDepth) < threshold;
                            if (!pixelBelow)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }
                            Boolean pixelBelowRight = Math.Abs(this.globalDepth[(y + 1) * depthWidth + x + 1] - currentDepth) < threshold;
                            if (!pixelBelowRight)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }


                            //outerPixels
                            Boolean pixelOuterTopLeft = Math.Abs(this.globalDepth[(y - 2) * depthWidth + x - 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterTopLeft)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterTopLeftMiddle = Math.Abs(this.globalDepth[(y - 2) * depthWidth + x - 1] - currentDepth) < outerThreshold;
                            if (!pixelOuterTopLeftMiddle)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterTopMiddle = Math.Abs(this.globalDepth[(y - 2) * depthWidth + x] - currentDepth) < outerThreshold;
                            if (!pixelOuterTopMiddle)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterTopRightMiddle = Math.Abs(this.globalDepth[(y - 2) * depthWidth + x + 1] - currentDepth) < outerThreshold;
                            if (!pixelOuterTopRightMiddle)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterTopRight = Math.Abs(this.globalDepth[(y + 2) * depthWidth + x + 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterTopRight)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }


                            Boolean pixelOuterBottomLeft = Math.Abs(this.globalDepth[(y + 2) * depthWidth + x - 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterBottomLeft)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterBottomLeftMiddle = Math.Abs(this.globalDepth[(y + 2) * depthWidth + x - 1] - currentDepth) < outerThreshold;
                            if (!pixelOuterBottomLeftMiddle)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterBottomMiddle = Math.Abs(this.globalDepth[(y + 2) * depthWidth + x] - currentDepth) < outerThreshold;
                            if (!pixelOuterBottomMiddle)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterBottomRightMiddle = Math.Abs(this.globalDepth[(y + 2) * depthWidth + x + 1] - currentDepth) < outerThreshold;
                            if (!pixelOuterBottomRightMiddle)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterBottomRight = Math.Abs(this.globalDepth[(y - 2) * depthWidth + x + 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterBottomRight)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }



                            Boolean pixelOuterleftTop = Math.Abs(this.globalDepth[(y - 1) * depthWidth + x - 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterleftTop)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterleft = Math.Abs(this.globalDepth[(y) * depthWidth + x - 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterleft)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterleftBottom = Math.Abs(this.globalDepth[(y + 1) * depthWidth + x - 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterleftBottom)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }



                            Boolean pixelOuterRightTop = Math.Abs(this.globalDepth[(y - 1) * depthWidth + x + 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterRightTop)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterRight = Math.Abs(this.globalDepth[(y) * depthWidth + x + 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterRight)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }

                            Boolean pixelOuterRightBottom = Math.Abs(this.globalDepth[(y + 1) * depthWidth + x + 2] - currentDepth) < outerThreshold;
                            if (!pixelOuterRightBottom)
                            {
                                this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                                continue;
                            }


                            this.globalDepthFilteredLookup[y * depthWidth + x] = true;
                        }
                        else
                        {
                            this.globalDepthFilteredLookup[y * depthWidth + x] = false;
                        }
                    }
                }
                this.depthFrameFiltered = true;
            }
            else {
                Console.WriteLine("Not filtered yet!!");
            }
        }

        private void filterDepthButton_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("filterDepthButton_Click");
            if (this.depthFrameCaptured)
            {
                this.filterDepth2(5);   
            }
            else
            {}
        }


        private void screenshot(int frameNumber, string subfolder, BitmapSource bitmap)
        {
            BitmapEncoder encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\kinect" + "\\" + subfolder;

            string path = Path.Combine(myPhotos, "frame" + frameNumber + ".png");

            // Write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }


            }
            catch (IOException)
            {

            }
        }

        private void runCalibration_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("runCalibration_Click");
            this.runningCalibration = true;
            this.captureDepthFrame = true;
        }

        private void makeScreenshotOfDepthMap() {
            if (this.depthFrameCaptured)
            {
                byte[] pixels = new byte[424 * 512 * 4];

                for (int y = 0; y < 424; y++)
                {
                    for (int x = 0; x < 512; x++)
                    {
                        if (this.globalDepthFilteredLookup[y * 512 + x])
                        {
                            pixels[(y * 512 + x) * 4] = 255;
                            pixels[(y * 512 + x) * 4 + 1] = 255;
                            pixels[(y * 512 + x) * 4 + 2] = 255;
                            pixels[(y * 512 + x) * 4 + 3] = 255;
                        }
                        else
                        {
                            pixels[(y * 512 + x) * 4] = 0;
                            pixels[(y * 512 + x) * 4 + 1] = 0;
                            pixels[(y * 512 + x) * 4 + 2] = 0;
                            pixels[(y * 512 + x) * 4 + 3] = 0;
                        }
                    }
                }


                var width = 512;
                var height = 424;
                var dpiX = 96;
                var dpiY = 96;
                var pixelFormat = PixelFormats.Pbgra32;
                var bytesPerPixel = (pixelFormat.BitsPerPixel + 7) / 8;
                var stride = (int)(bytesPerPixel * width);

                var result = BitmapSource.Create(width, height, dpiX, dpiY,
                                                 pixelFormat, null, pixels, stride);

                this.screenshot(0, "depthPicture", result);
                this.depthScreenshotTaken = true;
            }
            else {
                Console.WriteLine("No depth Frame captured yet!");
            }
        }

        private void captureDepthButton_Click(object sender, RoutedEventArgs e)
        {
            //Console.WriteLine("captureDepthButton_Click {0}", this.coordinateMapper.GetDepthCameraIntrinsics().RadialDistortionSecondOrder);

            if (this.depthFrameCaptured)
            {
                this.makeScreenshotOfDepthMap();
            }
            else {
                this.captureDepthFrame = true;
            }
        }

        public Point3D get3DPoint(Point depthPoint) {

            unsafe
            {
                //for (int i = 2000; i < 2020;i++ )
                //{

                //Console.WriteLine("DEPTHFRAME: {0}", frameData[512*424-1]);
                //}
                int depthWidth = 512;
                int depthHeight = 424;
                if (((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth < depthWidth * depthHeight && depthPoint.X > 0 && depthPoint.Y > 0)
                {
                    Console.WriteLine("lalala {0} , {1}", (((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth), this.globalDepth[((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth]);
                    ushort Z = this.globalDepth[((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth];
                    double X = (((double)Z) * (depthPoint.X - this.coordinateMapper.GetDepthCameraIntrinsics().PrincipalPointX)) / this.coordinateMapper.GetDepthCameraIntrinsics().FocalLengthX;
                    double Y = (((double)Z) * (depthPoint.Y - this.coordinateMapper.GetDepthCameraIntrinsics().PrincipalPointY)) / this.coordinateMapper.GetDepthCameraIntrinsics().FocalLengthY;

                    return new Point3D( X, Y, Z);
                }
               

            }

            return new Point3D(-1,-1,-1);
        }

        private void startCalculating() {
            Console.WriteLine("startCalculating()");
            this.calculatingStarted = true;
            if (this.verticalProjectingDone && this.horizontalProjectingDone)
            {
                this.calculateButton.IsEnabled = false;
                CorrespondancesBuilder cb = new CorrespondancesBuilder(this.verticalImages, this.horizontalImages, this.projectorWidth, this.projectorHeight);
                //cb.calculateCorrespondances();
                cb.calculateHorizontalCorrespondances();
                Console.WriteLine("printTestCorrespondances");
                cb.printTestCorrespondances(this.globalDepth, this.coordinateMapper, this.colorMappedToDepthPoints, this.globalDepthFilteredLookup);
            }
        }

        private void calculateButton_Click(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("calculateButton_Click");
            //this.writeImageToFirstImageBuffer = true;
            if(this.verticalProjectingDone && this.horizontalProjectingDone){
                this.calculateButton.IsEnabled = false;
                CorrespondancesBuilder cb = new CorrespondancesBuilder(this.verticalImages, this.horizontalImages, this.projectorWidth, this.projectorHeight);
                //cb.calculateCorrespondances();
                cb.calculateHorizontalCorrespondances();
                Console.WriteLine("printTestCorrespondances");
                cb.printTestCorrespondances(this.globalDepth,this.coordinateMapper, this.colorMappedToDepthPoints, this.globalDepthFilteredLookup);
            }
            
        }


        /// <summary>
        /// Handles the user clicking on the screenshot button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void ScreenshotButton_Click(object sender, RoutedEventArgs e)
        {
            // Create a render target to which we'll render our composite image
            RenderTargetBitmap renderBitmap = new RenderTargetBitmap((int)CompositeImage.ActualWidth, (int)CompositeImage.ActualHeight, 96.0, 96.0, PixelFormats.Pbgra32);

            DrawingVisual dv = new DrawingVisual();
            using (DrawingContext dc = dv.RenderOpen())
            {
                VisualBrush brush = new VisualBrush(CompositeImage);
                dc.DrawRectangle(brush, null, new Rect(new Point(), new Size(CompositeImage.ActualWidth, CompositeImage.ActualHeight)));
            }

            renderBitmap.Render(dv);

            BitmapEncoder encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(renderBitmap));

            unsafe
            {
                uint* bitmapPixelsPointer = (uint*)this.bitmap.BackBuffer;
                uint packedColor = bitmapPixelsPointer[0];

                Color unpackedColor = new Color();
                unpackedColor.B = (byte)(packedColor);
                unpackedColor.G = (byte)(packedColor >> 8);
                unpackedColor.R = (byte)(packedColor >> 16);
                unpackedColor.A = (byte)(packedColor >> 24);


                Console.WriteLine("Bitmap [0][0] Element: B:{0}, G:{1}, R:{2}, A:{3}", unpackedColor.B, unpackedColor.G, unpackedColor.R, unpackedColor.A);
            }

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures)+"\\kinect";

            string path = Path.Combine(myPhotos, "KinectScreenshot-CoordinateMapping-" + time + ".png");

            // Write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }

                this.StatusText = string.Format(Properties.Resources.SavedScreenshotStatusTextFormat, path);
            }
            catch (IOException)
            {
                this.StatusText = string.Format(Properties.Resources.FailedScreenshotStatusTextFormat, path);
            }
        }


        private void screenshot(int frameNumber, string subfolder)
        {
            BitmapEncoder encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(this.bitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\kinect"+"\\"+subfolder;

            string path = Path.Combine(myPhotos, "frame" + frameNumber + ".png");

            // Write the new file to disk
            try
            {
                using (FileStream fs = new FileStream(path, FileMode.Create))
                {
                    encoder.Save(fs);
                }

                this.StatusText = string.Format(Properties.Resources.SavedScreenshotStatusTextFormat, path);
            }
            catch (IOException)
            {
                this.StatusText = string.Format(Properties.Resources.FailedScreenshotStatusTextFormat, path);
            }
        }

        private Point getDepthPoint(Point cameraPixel) {

            int colorMappedToDepthPointCount = this.colorMappedToDepthPoints.Length;
            unsafe {
                fixed(DepthSpacePoint* colorMappedToDepthPointsPointer = this.colorMappedToDepthPoints){

                    int cameraWidth = 1920;
                    int cameraHeight = 1080;

                    float colorMappedToDepthX = colorMappedToDepthPointsPointer[((int)cameraPixel.X) + ((int)cameraPixel.Y) * cameraWidth].X;
                    float colorMappedToDepthY = colorMappedToDepthPointsPointer[((int)cameraPixel.X) + ((int)cameraPixel.Y) * cameraWidth].Y;

                    int depthWidth = 512;
                    int depthHeight = 424;


                    if (!float.IsNegativeInfinity(colorMappedToDepthX) &&
                                    !float.IsNegativeInfinity(colorMappedToDepthY))
                    {
                        // Make sure the depth pixel maps to a valid point in color space
                        int depthX = (int)(colorMappedToDepthX + 0.5f);
                        int depthY = (int)(colorMappedToDepthY + 0.5f);

                        return new Point(depthX, depthY);

                        // If the point is not valid, there is no body index there.
                        /*if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                        {
                            int depthIndex = (depthY * depthWidth) + depthX;
                        }*/
                    }
                }
            }
            return new Point(-1, -1);
        }



        /// <summary>
        /// Handles the depth/color/body index frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {

            if(this.runningCalibration){
                //Console.WriteLine("runningCalibration"); 
                if(this.depthFrameCaptured){
                    //Console.WriteLine("depthFrameCaptured");
                    if(this.depthFrameFiltered){
                        //Console.WriteLine("this.depthFrameFiltered");
                        if (this.depthScreenshotTaken)
                        {
                            //Console.WriteLine("this.depthScreenshotTaken");

                                if (this.horizontalProjectingDone && this.verticalProjectingDone)
                                {
                                    this.StatusText = "this.horizontalProjectingDone && this.verticalProjectingDone";
                                    if (this.calculatingStarted)
                                    {
                                        this.StatusText = "calculatingFinished";
                                    }
                                    else {
                                        this.StatusText = "calculating";
                                        this.startCalculating();
                                    }
                                }
                                else { 
                                    //this.StatusText = "projecting Patterns";
                                this.writeImageToFirstImageBuffer = true;
                                }
                            //}
                            /*else {
                                this.StatusText = "projecting Patterns";
                                this.writeImageToFirstImageBuffer = true;
                            }*/
                        }
                        else {
                            this.StatusText = "making Screenshot of DepthMap";
                            this.makeScreenshotOfDepthMap();
                        }
                    }
                    else{
                        this.StatusText = "filtering Depth";
                        this.filterDepth2(5);
                    }
                }
            }

            int depthWidth = 0;
            int depthHeight = 0;
                    
            DepthFrame depthFrame = null;
            ColorFrame colorFrame = null;
            BodyIndexFrame bodyIndexFrame = null;
            bool isBitmapLocked = false;

            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();           

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {                
                depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame();
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                bodyIndexFrame = multiSourceFrame.BodyIndexFrameReference.AcquireFrame();

                // If any frame has expired by the time we process this event, return.
                // The "finally" statement will Dispose any that are not null.
                if ((depthFrame == null) || (colorFrame == null) || (bodyIndexFrame == null))
                {
                    return;
                }

                // Process Depth
                FrameDescription depthFrameDescription = depthFrame.FrameDescription;

                depthWidth = depthFrameDescription.Width;
                depthHeight = depthFrameDescription.Height;

                // Access the depth frame data directly via LockImageBuffer to avoid making a copy
                using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                {
                    this.coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(
                        depthFrameData.UnderlyingBuffer,
                        depthFrameData.Size,
                        this.colorMappedToDepthPoints);
                    if(this.captureDepthFrame){

                        unsafe {                   
                            depthFrame.CopyFrameDataToArray(this.globalDepth);
                            //for (int i = 2000; i < 2020;i++ )
                            //{
                            this.minDepthRange = depthFrame.DepthMinReliableDistance;
                            this.maxDepthRange = depthFrame.DepthMaxReliableDistance;
                            //Console.WriteLine("DEPTHFRAME: {0}", frameData[512*424-1]);
                            //}
                            
                        }

                        this.captureDepthFrame = false;
                        this.depthFrameCaptured = true;
                    }
                }

                // We're done with the DepthFrame 
                depthFrame.Dispose();
                depthFrame = null;

                // Process Color

                // Lock the bitmap for writing
                this.bitmap.Lock();
                isBitmapLocked = true;

                colorFrame.CopyConvertedFrameDataToIntPtr(this.bitmap.BackBuffer, this.bitmapBackBufferSize, ColorImageFormat.Bgra);

                if (this.projectingStarted && !this.verticalProjectingDone)
                {
                    Boolean newPattern = false;
                    bool timeToTakeScreenshot = false;
                    if (this.frameCounter==10)
                    {
                        timeToTakeScreenshot = true;
                    }
                    if (this.frameCounter >= 15)
                    {
                        newPattern = true;
                        this.projectionImageCounter++;
                        this.frameCounter = 0;
                    }
                    else {
                        this.frameCounter++;
                    }

                    if (timeToTakeScreenshot)
                    {
                        //int size = this.verticalImages.Count();
                        this.bitmap.Unlock();
                        this.screenshot(this.projectionImageCounter,"horizontal");
                        this.bitmap.Lock();
                        //this.verticalImages[size]
                        timeToTakeScreenshot = false;
                    }

                    if (newPattern)
                    {

                        ProjectablePattern pp = new ProjectablePattern(this.projectorWidth, this.projectorHeight, true, this.projectionImageCounter, this.myProjector);
                        bool success = pp.projectPattern();
                        if (!success)
                        {
                            this.frameCounter = 0;
                            this.projectionImageCounter = 0;
                            this.verticalProjectingDone = true;
                        }
                        else {
                            this.horizontalImages.Add(pp);
                        }
                        

                    }
                }



                if (this.projectingStarted && this.verticalProjectingDone && !this.horizontalProjectingDone)
                {
                    Boolean newPattern = false;
                    bool timeToTakeScreenshot = false;
                    if (this.frameCounter == 10)
                    {
                        timeToTakeScreenshot = true;
                    }
                    if (this.frameCounter >= 15)
                    {
                        newPattern = true;
                        this.projectionImageCounter++;
                        this.frameCounter = 0;
                    }
                    else
                    {
                        this.frameCounter++;
                    }

                    if (timeToTakeScreenshot)
                    {
                        //int size = this.verticalImages.Count();
                        this.bitmap.Unlock();
                        this.screenshot(this.projectionImageCounter, "vertical");
                        this.bitmap.Lock();
                        //this.verticalImages[size]
                        timeToTakeScreenshot = false;
                    }

                    if (newPattern)
                    {

                        ProjectablePattern pp = new ProjectablePattern(this.projectorWidth, this.projectorHeight, false, this.projectionImageCounter, this.myProjector);
                        bool success = pp.projectPattern();
                        if (!success)
                        {
                            this.horizontalProjectingDone = true;
                            this.projectButton.IsEnabled = false;
                        }
                        else {
                            this.verticalImages.Add(pp);
                        }
                        

                    }
                }
                
                
                if (this.writeImageToFirstImageBuffer)
                {
                    colorFrame.CopyConvertedFrameDataToIntPtr(this.firstImage.BackBuffer, this.bitmapBackBufferSize, ColorImageFormat.Bgra);
                    this.writeImageToFirstImageBuffer = false;
                    this.projectingStarted = true;
                }

                // We're done with the ColorFrame 
                colorFrame.Dispose();
                colorFrame = null;

                // We'll access the body index data directly to avoid a copy
                using (KinectBuffer bodyIndexData = bodyIndexFrame.LockImageBuffer())
                {
                    unsafe
                    {
                        byte* bodyIndexDataPointer = (byte*)bodyIndexData.UnderlyingBuffer;

                        int colorMappedToDepthPointCount = this.colorMappedToDepthPoints.Length;

                        fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = this.colorMappedToDepthPoints)
                        {
                            // Treat the color data as 4-byte pixels
                            uint* bitmapPixelsPointer = (uint*)this.bitmap.BackBuffer;
                            uint* firstImagePixelsPointer = (uint*)this.firstImage.BackBuffer;

                            // Loop over each row and column of the color image
                            // Zero out any pixels that don't correspond to a body index
                            for (int colorIndex = 0; colorIndex < colorMappedToDepthPointCount; ++colorIndex)
                            {
                                float colorMappedToDepthX = colorMappedToDepthPointsPointer[colorIndex].X;
                                float colorMappedToDepthY = colorMappedToDepthPointsPointer[colorIndex].Y;

                                if (colorIndex==600)
                                {
                                    
                                

                                // The sentinel value is -inf, -inf, meaning that no depth pixel corresponds to this color pixel.
                                if (!float.IsNegativeInfinity(colorMappedToDepthX) &&
                                    !float.IsNegativeInfinity(colorMappedToDepthY))
                                {
                                    /*Console.WriteLine("colorMappedToDepthX: {0}, colorMappedToDepthY {1} ,colorIndex: {2}", colorMappedToDepthX, colorMappedToDepthY, colorIndex);*/
                                    // Make sure the depth pixel maps to a valid point in color space
                                    int depthX = (int)(colorMappedToDepthX + 0.5f);
                                    int depthY = (int)(colorMappedToDepthY + 0.5f);

                                    // If the point is not valid, there is no body index there.
                                    if ((depthX >= 0) && (depthX < depthWidth) && (depthY >= 0) && (depthY < depthHeight))
                                    {
                                        int depthIndex = (depthY * depthWidth) + depthX;

                                        // If we are tracking a body for the current pixel, do not zero out the pixel
                                        if (bodyIndexDataPointer[depthIndex] != 0xff)
                                        {
                                            continue;
                                        }
                                    }
                                }

                                }//(colorIndex<200)

                                //bitmapPixelsPointer[colorIndex] = 0;
                            }
                        }

                        this.bitmap.AddDirtyRect(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight));

                    }
                }
            }
            finally
            {
                if (isBitmapLocked)
                {
                    this.bitmap.Unlock();
                }

                if (depthFrame != null)
                {
                    depthFrame.Dispose();
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }

                if (bodyIndexFrame != null)
                {
                    bodyIndexFrame.Dispose();
                }
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }
}
