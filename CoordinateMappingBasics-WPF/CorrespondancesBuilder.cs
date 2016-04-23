using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;

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
using System.Drawing;
using System.Drawing.Imaging;

namespace Microsoft.Samples.Kinect.CoordinateMappingBasics
{
    class CorrespondancesBuilder
    {
        private List<ProjectablePattern> verticalImages;
        private List<ProjectablePattern> horizontalImages;
        private List<BitmapSource> horizontalScreenshots;
        private List<BitmapSource> verticalScreenshots;
        private List<BitmapSource> verticalSubtractions;
        private List<BitmapSource> horizontalSubtractions;

        private int THRESHOLD = 35;
        private int projectorWidth=0;
        private int projectorHeight = 0;

        public CorrespondancesBuilder(List<ProjectablePattern> verticalImages, List<ProjectablePattern> horizontalImages, int projectorWidth, int projectorHeight)
        {
            this.verticalImages = verticalImages;
            this.horizontalImages = horizontalImages;
            this.horizontalScreenshots = new List<BitmapSource>();
            this.verticalScreenshots = new List<BitmapSource>();
            this.verticalSubtractions = new List<BitmapSource>();
            this.horizontalSubtractions = new List<BitmapSource>();
            this.projectorWidth = projectorWidth;
            this.projectorHeight = projectorHeight;

            for (int i = 1; i < this.horizontalImages.Count; i++)
            {
                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "horizontal";
                bool exists = System.IO.Directory.Exists(myPhotos);
                if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
                string path = Path.Combine(myPhotos, "frame" + i + ".png");
                BitmapSource bitmapSource = this.getBitmapFrame(path);
                this.horizontalScreenshots.Add(bitmapSource);
            }

            for (int i = 1; i < this.verticalImages.Count; i++)
            {
                string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "vertical";
                bool exists = System.IO.Directory.Exists(myPhotos);
                if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
                string path = Path.Combine(myPhotos, "frame" + i + ".png");
                BitmapSource bitmapSource = this.getBitmapFrame(path);
                this.verticalScreenshots.Add(bitmapSource);
            }

            //Console.WriteLine("verticalImages: {0}, horizontalImages: {1}, horizontalScreenshots: {2}, verticalScreenshots: {3} ", this.verticalImages.Count, this.horizontalImages.Count, this.horizontalScreenshots.Count, this.verticalScreenshots.Count);

        }


        private BitmapSource getBitmapFrame(String path) {
            Stream imageStreamSource = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read);
            PngBitmapDecoder decoder = new PngBitmapDecoder(imageStreamSource, BitmapCreateOptions.PreservePixelFormat, BitmapCacheOption.Default);
            BitmapSource bitmapSource = decoder.Frames[0];

            return bitmapSource;
        }

        /*private unsafe System.Windows.Media.Color GetPixelColor(BitmapSource bitmap, int x, int y)
        {
            System.Windows.Media.Color color;
            var bytesPerPixel = (bitmap.Format.BitsPerPixel + 7) / 8;
            var bytes = new byte[bytesPerPixel];
            var rect = new Int32Rect(x, y, 1, 1);

            bitmap.CopyPixels(rect, bytes, bytesPerPixel, 0);

            if (bitmap.Format == PixelFormats.Pbgra32)
            {
                color = System.Windows.Media.Color.FromArgb(bytes[3], bytes[2], bytes[1], bytes[0]);
            }
            else if (bitmap.Format == PixelFormats.Bgr32)
            {
                color = System.Windows.Media.Color.FromArgb(0xFF, bytes[2], bytes[1], bytes[0]);
            }
            // handle other required formats
            else
            {
                color = Colors.Black;
            }

            return color;
        }*/


        public List<System.Windows.Point> getPoints(System.Windows.Point input)
        {

            List<System.Windows.Point> myPoints = new List<System.Windows.Point>();

            if (this.verticalImages[11].pixelIsWhite(((int)input.X), ((int)input.Y)))
            {
                myPoints = this.listLightedPixels(verticalSubtractions[11]);
                //Console.WriteLine("myPoints.Count1: {0}", myPoints.Count);
            }
            else {
                myPoints = this.listLightedPixels(verticalSubtractions[10]);
                //Console.WriteLine("myPoints.Count1: {0}", myPoints.Count);
            }

            for (int i = 0; i < verticalSubtractions.Count; i++)
            {
                if (this.verticalImages[i].pixelIsWhite(((int)input.X), ((int)input.Y)) && this.verticalImages[i].getDistanceOfPixelToBorder(((int)input.X), ((int)input.Y)) >= 1)
                {
                    myPoints = this.filterLightedPixels(verticalSubtractions[i], myPoints);
                    //Console.WriteLine("myPoints.CountA{0}: {1}",i, myPoints.Count);
                }
            }

            for (int i = 0; i < horizontalSubtractions.Count; i++)
            {
                if (this.horizontalImages[i].pixelIsWhite(((int)input.X), ((int)input.Y)) && this.horizontalImages[i].getDistanceOfPixelToBorder(((int)input.X), ((int)input.Y)) >= 1)
                {
                    myPoints = this.filterLightedPixels(horizontalSubtractions[i], myPoints);
                    //Console.WriteLine("myPoints.CountB{0}: {1}", i, myPoints.Count);
                }
            }

            /*for (int i = 0; i < myPoints.Count; i++)
            {
                Console.WriteLine("myPoints{0}: {1}", i, myPoints[i]);
            }*/

            return myPoints;
        }


        public List<List<System.Windows.Point>> getPoints2(List<System.Windows.Point> input)
        {

            List<List<System.Windows.Point>> myPoints = new List<List<System.Windows.Point>>();

            for (int a = 0; a < input.Count;a++ )
            {
                //Console.WriteLine("myPoints.Count1: {0}, {1}, {2}", a, (int)input[a].X, (int)input[a].Y);
                //Console.WriteLine("myPoints.Count1: {0}", myPoints.Count);
                if (this.verticalImages[11].pixelIsWhite(((int)input[a].X), ((int)input[a].Y)))
                {
                    myPoints.Add(this.listLightedPixels(verticalSubtractions[11]));
                    //Console.WriteLine("myPoints.Count1: {0}", myPoints.Count);
                }
                else
                {
                    myPoints.Add(this.listLightedPixels(verticalSubtractions[10]));
                    //Console.WriteLine("myPoints.Count1: {0}", myPoints.Count);
                }
            }

                for (int i = 0; i < verticalSubtractions.Count; i++)
                {

                        myPoints = this.filterLightedPixels2(verticalSubtractions[i], myPoints, verticalImages[i],input);
                        //Console.WriteLine("myPoints.CountA{0}: {1}",i, myPoints.Count);

                }

                for (int i = 0; i < horizontalSubtractions.Count; i++)
                {

                        myPoints = this.filterLightedPixels2(horizontalSubtractions[i], myPoints, horizontalImages[i],input);

                }
            return myPoints;
        }


        private Point3D get3DPoint(System.Windows.Point depthPoint, ushort[] depthPoints, CoordinateMapper coordinateMapper)
        {

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
                    //Console.WriteLine("lalala {0} , {1}", (((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth), depthPoints[((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth]);
                    double Z = (-1) * depthPoints[((int)depthPoint.X) + ((int)depthPoint.Y) * depthWidth];
                    double X = (((double)Z) * (depthPoint.X - coordinateMapper.GetDepthCameraIntrinsics().PrincipalPointX)) / coordinateMapper.GetDepthCameraIntrinsics().FocalLengthX;
                    double Y = (((double)Z) * (depthPoint.Y - coordinateMapper.GetDepthCameraIntrinsics().PrincipalPointY)) / coordinateMapper.GetDepthCameraIntrinsics().FocalLengthY;

                    return new Point3D(X, Y, Z);
                }


            }

            return new Point3D(-1, -1, -1);
        }


        private void writeResultsToFile(List<System.Windows.Point> beamerPixels, List<Point3D> points3d)
        {

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration" + "\\" + "results";
            bool exists = System.IO.Directory.Exists(myPhotos);
            if (!exists) System.IO.Directory.CreateDirectory(myPhotos);

            string path = Path.Combine(myPhotos, "first_results_1.txt");

            TextWriter tw = new StreamWriter(path);

            // write a line of text to the file

            for(int i=0;i < beamerPixels.Count;i++){
                tw.WriteLine("{0} {1} {2} {3} {4}", ((int)beamerPixels[i].X), ((int)beamerPixels[i].Y), points3d[i].X, points3d[i].Y, points3d[i].Z);
            }

            

            // close the stream
            tw.Close();
        }


        public void printTestCorrespondances(ushort[] globalDepth, CoordinateMapper coordinateMapper, DepthSpacePoint[] colorMappedToDepthPoints, Boolean[] globalDepthFilteredLookup)
        {

            int counter = 0;
            //Console.WriteLine("inside printTestCorrespondances");

            List<System.Windows.Point> beamerPixels = new List<System.Windows.Point>();
            List<Point3D> points3d = new List<Point3D>();

            List<System.Windows.Point> myRandomInputPoints = this.getInputPoints(200);//new List<System.Windows.Point>();

            List<List<System.Windows.Point>> projector2dCoordinatesInCamera2d = new List<List<System.Windows.Point>>();

            List<List<System.Windows.Point>> errors = new List<List<System.Windows.Point>>();

            /*for (int i = 0; i < 8;i++)
            {
                List<System.Windows.Point> myRandomInputPointsBuffer = this.getInputPoints(15);
                myRandomInputPoints.AddRange(myRandomInputPointsBuffer);
                List<List<System.Windows.Point>> projector2dCoordinatesInCamera2dBuffer = this.getPoints2(myRandomInputPointsBuffer);
                projector2dCoordinatesInCamera2d.AddRange(projector2dCoordinatesInCamera2dBuffer);
            }*/

            //this.getInputPoints(300);

            List<System.Windows.Point> camera2dResult = new List<System.Windows.Point>();

            for (int i = 0;i< myRandomInputPoints.Count; i++)
            {
                List<System.Windows.Point> camera2DPoints = this.getPoints(myRandomInputPoints[i]);//projector2dCoordinatesInCamera2d[i];//this.getPoints(myRandomInputPoints[i]);
                //Console.WriteLine("Projector2d: {0} ", myRandomInputPoints[i]);
                if (this.checkResults(camera2DPoints))
                {

                    //Console.WriteLine("this.checkResults(camera2DPoints)");

                    for (int j = 0; j < camera2DPoints.Count; j++)
                    {
                        unsafe
                        {

                            fixed (DepthSpacePoint* colorMappedToDepthPointsPointer = colorMappedToDepthPoints)
                            {

                                int cameraWidth = 1920;

                                float colorMappedToDepthX = colorMappedToDepthPointsPointer[((int)camera2DPoints[j].X) + ((int)camera2DPoints[j].Y) * cameraWidth].X;
                                float colorMappedToDepthY = colorMappedToDepthPointsPointer[((int)camera2DPoints[j].X) + ((int)camera2DPoints[j].Y) * cameraWidth].Y;

                                if (float.IsNegativeInfinity(colorMappedToDepthX) ||
                                   float.IsNegativeInfinity(colorMappedToDepthY))
                                {
                                    Console.WriteLine("Filtered neg inf {0},{1}", colorMappedToDepthX, colorMappedToDepthY);
                                    continue;
                                }


                                if (!globalDepthFilteredLookup[(int)(colorMappedToDepthY * 512 + colorMappedToDepthX)])
                                {
                                    Console.WriteLine("Filtered {0},{1}", colorMappedToDepthX, colorMappedToDepthY);
                                    continue;
                                }
                                System.Windows.Point depthPoint = new System.Windows.Point(colorMappedToDepthX, colorMappedToDepthY);
                                Point3D coordinates3d = this.get3DPoint(depthPoint, globalDepth, coordinateMapper);

                                if (j == camera2DPoints.Count - 1)
                                {
                                    beamerPixels.Add(new System.Windows.Point(myRandomInputPoints[i].X, myRandomInputPoints[i].Y));
                                    points3d.Add(new Point3D(coordinates3d.X, coordinates3d.Y, coordinates3d.Z));
                                    camera2dResult.Add(camera2DPoints[j]);

                                    counter++;
                                }

                            }


                        }

                    }
                }
                else {
                    if (camera2DPoints.Count>1)
                    {
                        errors.Add(this.twoPointsWithGreatestDistance(camera2DPoints));
                    }  
                }
            }

            this.makePhotoThatDiplaysCorrespondances(camera2dResult, errors);
            this.writeResultsToFile(beamerPixels, points3d);
            Console.WriteLine("Number of Results: {0}", counter);
            string myPoints = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration" + "\\" + "results";
            bool exists = System.IO.Directory.Exists(myPoints);
            if (!exists) System.IO.Directory.CreateDirectory(myPoints);


            string path = Path.Combine(myPoints, "numberOfPoints.txt");

            TextWriter tw = new StreamWriter(path);

            // write a line of text to the file

            tw.WriteLine("{0}", counter);
            tw.Close();
            

        }

        private void makePhotoThatDiplaysCorrespondances(List<System.Windows.Point> camera2dResult, List<List<System.Windows.Point>> errors)
        {
            Bitmap myBitmap = this.getBitmap(this.horizontalScreenshots[0]);
            for (int i = 0; i < camera2dResult.Count;i++ )
            {
                for (int j = 0; j < 9;j++ )
                {
                    int pixelX = (int)camera2dResult[i].X-1+j%3;
                    int pixelY = (int)camera2dResult[i].Y-1+j/3;
                    myBitmap.SetPixel(pixelX, pixelY, System.Drawing.Color.Green);
                }
            }

            for (int i = 0; i < errors.Count; i++)
            {
                for (int j = 0; j < 9; j++)
                {
                    int pixelX = (int)errors[i][0].X - 1 + j % 3;
                    int pixelY = (int)errors[i][0].Y - 1 + j / 3;
                    myBitmap.SetPixel(pixelX, pixelY, System.Drawing.Color.Red);
                }

                for (int j = 0; j < 9; j++)
                {
                    int pixelX = (int)errors[i][1].X - 1 + j % 3;
                    int pixelY = (int)errors[i][1].Y - 1 + j / 3;
                    myBitmap.SetPixel(pixelX, pixelY, System.Drawing.Color.Red);
                }
            }

            string picturePath = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\results";
            bool exists = System.IO.Directory.Exists(picturePath);
            if (!exists) System.IO.Directory.CreateDirectory(picturePath);
            string path = Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments) + "\\Github\\masterThesis\\results\\calibrationPointsLocation.png";
            myBitmap.Save(path);
            myBitmap.Dispose();
        }

        public double EuclideanDistance(System.Windows.Point p1, System.Windows.Point p2) {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }

        public List<System.Windows.Point> twoPointsWithGreatestDistance(List<System.Windows.Point> inputPoints) {

            List<System.Windows.Point> result = new List<System.Windows.Point>();
            double maxEuclidDistance = 0.0;
            int indexOne = 0;
            int indexTwo = 0;
            for (int i = 0; i < inputPoints.Count; i++)
            {
                for (int j = i; j < inputPoints.Count;j++ )
                {
                    if (this.EuclideanDistance(inputPoints[i], inputPoints[j]) > maxEuclidDistance)
                    {
                        maxEuclidDistance = this.EuclideanDistance(inputPoints[i], inputPoints[j]);
                        indexOne = i;
                        indexTwo = j;
                    }
                }
            }

            result.Add(inputPoints[indexOne]);
            result.Add(inputPoints[indexTwo]);

            return result;
        }

        public bool checkResults(List<System.Windows.Point> inputPoints)
        {

            if (inputPoints.Count < 2)
            {
                Console.WriteLine("CheckResults: Less than two input Points");
                return false;
            }

            int xMin = (int) inputPoints[0].X;
            int yMin = (int)inputPoints[0].Y;
            int xMax = (int)inputPoints[0].X;
            int yMax = (int)inputPoints[0].Y;
            for(int i=1 ; i<inputPoints.Count; i++){
                xMin = Math.Min(xMin, (int)inputPoints[i].X);
                yMin = Math.Min(yMin, (int)inputPoints[i].Y);
                xMax = Math.Max(xMax, (int)inputPoints[i].X);
                yMax = Math.Max(yMax, (int)inputPoints[i].Y);
            }

            if (xMax - xMin <= 6 && yMax - yMin <= 6)
            {
                return true;
            }
            else {
                Console.WriteLine("CheckResults: xMax: {0} xMin: {1} yMax: {2} yMin: {3}",xMax,xMin,yMax,yMin);
                return false;
            }
        }

        private bool isGoodNumber(int input) {

            if(input % 5 ==0){
                return false;
            }

            if (input % 2 == 0)
            {
                return false;
            }

            return true;
        }

        public List<System.Windows.Point> getInputPoints(int length)
        {
            List<System.Windows.Point> result = new List<System.Windows.Point>();
            Random rnd = new Random();
            

            while (result.Count<length)
            {           
                int x = rnd.Next(0, this.projectorWidth);
                while(!this.isGoodNumber(x)){
                    x = rnd.Next(0, this.projectorWidth); ;
                }
                int y = rnd.Next(0, this.projectorHeight);
                while (!this.isGoodNumber(y))
                {
                    y = rnd.Next(0, this.projectorHeight); ;
                }
                /*if(result.Contains(new Point(x,y))){
                    continue;
                }*/
                int verticalStrikes = 0;
                int horizontalStrikes = 0;
                for (int i = 2; i < verticalSubtractions.Count; i++) {
                    if (this.verticalImages[i].pixelIsWhite(x,y) && !(this.verticalImages[i].getDistanceOfPixelToBorder(x,y) >= 2))
                    {
                        verticalStrikes++;
                    }
                }

                for (int i = 2; i < horizontalSubtractions.Count; i++)
                {
                    if (this.horizontalImages[i].pixelIsWhite(x, y) && !(this.horizontalImages[i].getDistanceOfPixelToBorder(x, y) >= 2))
                    {
                        horizontalStrikes++;
                    }
                }

                if(verticalStrikes<4 && horizontalStrikes<4){
                    result.Add(new System.Windows.Point(x, y));
                }

            }

            return result;
        }

        public void calculateHorizontalCorrespondances() {

            Console.WriteLine("this.verticalImages.Count {0}", this.verticalImages.Count);

            for (int i = 1; i < this.horizontalImages.Count; i++)
            {
                if (i % 2 == 1)
                {
                    string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "horizontal";
                    bool exists = System.IO.Directory.Exists(myPhotos);
                    if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
                    string path = Path.Combine(myPhotos, "frame" + i + ".png");
                    BitmapSource bitmapSource1 = this.getBitmapFrame(path);

                    path = Path.Combine(myPhotos, "frame" + (i + 1) + ".png");
                    BitmapSource bitmapSource2 = this.getBitmapFrame(path);

                    var result = this.subtractTwoBitmapSources(bitmapSource2, bitmapSource1, THRESHOLD, -300);
                    horizontalSubtractions.Add(result);
                    this.screenshot(i, "testHori", result);
                }
                else {
                    string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "horizontal";
                    bool exists = System.IO.Directory.Exists(myPhotos);
                    if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
                    string path = Path.Combine(myPhotos, "frame" + i + ".png");
                    BitmapSource bitmapSource1 = this.getBitmapFrame(path);

                    path = Path.Combine(myPhotos, "frame" + (i - 1) + ".png");
                    BitmapSource bitmapSource2 = this.getBitmapFrame(path);

                    var result = this.subtractTwoBitmapSources(bitmapSource2, bitmapSource1, THRESHOLD, -300);
                    horizontalSubtractions.Add(result);
                    this.screenshot(i, "testHori", result);
                }
            }

            for (int i = 1; i < this.verticalImages.Count ; i++)
            {
                if (i % 2 == 1)
                {
                    string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "vertical";
                    bool exists = System.IO.Directory.Exists(myPhotos);
                    if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
                    string path = Path.Combine(myPhotos, "frame" + i + ".png");
                    BitmapSource bitmapSource1 = this.getBitmapFrame(path);

                    path = Path.Combine(myPhotos, "frame" + (i + 1) + ".png");
                    BitmapSource bitmapSource2 = this.getBitmapFrame(path);

                    var result = this.subtractTwoBitmapSources(bitmapSource2, bitmapSource1, THRESHOLD, -300);
                    verticalSubtractions.Add(result);
                    this.screenshot(i, "testVert", result);
                }
                else {
                    string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "vertical";
                    bool exists = System.IO.Directory.Exists(myPhotos);
                    if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
                    string path = Path.Combine(myPhotos, "frame" + i + ".png");
                    BitmapSource bitmapSource1 = this.getBitmapFrame(path);

                    path = Path.Combine(myPhotos, "frame" + (i - 1) + ".png");
                    BitmapSource bitmapSource2 = this.getBitmapFrame(path);

                    var result = this.subtractTwoBitmapSources(bitmapSource2, bitmapSource1, THRESHOLD, -300);//55 picop
                    verticalSubtractions.Add(result);
                    this.screenshot(i, "testVert", result);
                }
            }

        }

        private BitmapSource convertByteArrayToBitmapSource(BitmapSource bitmapSource, byte[] pixels)
        {
            var width = (int)bitmapSource.Width;
            var height = (int)bitmapSource.Height;
            var dpiX = bitmapSource.DpiX;
            var dpiY = bitmapSource.DpiY;
            var pixelFormat = PixelFormats.Pbgra32;
            var bytesPerPixel = (pixelFormat.BitsPerPixel + 7) / 8;
            var stride = (int)(bytesPerPixel * width);

            var result = BitmapSource.Create(width, height, dpiX, dpiY,
                                             pixelFormat, null, pixels, stride);
            return result;
        }

        private BitmapSource subtractTwoBitmapSources(BitmapSource bitmapSource1, BitmapSource bitmapSource2, int cutoff1, int cutoff2)
        {
            int stride1 = bitmapSource1.PixelWidth * 4;
            int size1 = bitmapSource1.PixelHeight * stride1;
            byte[] pixels1 = new byte[size1];
            bitmapSource1.CopyPixels(pixels1, stride1, 0);

            int stride2 = bitmapSource2.PixelWidth * 4;
            int size2 = bitmapSource2.PixelHeight * stride1;
            byte[] pixels2 = new byte[size2];
            bitmapSource2.CopyPixels(pixels2, stride2, 0);


            byte[] pixels3 = new byte[size2];
            for (int i = 0; i < pixels2.Length; i++)
            {
                if (i % 4 != 3)
                {

                    byte a = pixels2[i];
                    byte b = pixels1[i];
                    int temp = a - b;
                    if (temp < cutoff1 && temp > cutoff2)
                    {
                        pixels3[i] = 0;
                    }
                    else
                    {
                        pixels3[i] = (byte)(temp);
                    }

                }
                else
                {
                    pixels3[i] = pixels1[i];
                }
            }

            var result = this.convertByteArrayToBitmapSource(bitmapSource1, pixels3);

            return result;
            
        }

        private Bitmap getBitmap(BitmapSource source)
        {
            Bitmap bmp = new Bitmap(
              source.PixelWidth,
              source.PixelHeight,
              System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
            BitmapData data = bmp.LockBits(
              new Rectangle(System.Drawing.Point.Empty, bmp.Size),
              ImageLockMode.WriteOnly,
              System.Drawing.Imaging.PixelFormat.Format32bppPArgb);
            source.CopyPixels(
              Int32Rect.Empty,
              data.Scan0,
              data.Height * data.Stride,
              data.Stride);
            bmp.UnlockBits(data);
            return bmp;
        }


        private unsafe List<System.Windows.Point> detectColorWithUnsafe(Bitmap image)
        {
            List<System.Windows.Point> result = new List<System.Windows.Point>();

            BitmapData imageData = image.LockBits(new Rectangle(0, 0, image.Width,
              image.Height), ImageLockMode.ReadWrite, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            int bytesPerPixel = 3;

            byte* scan0 = (byte*)imageData.Scan0.ToPointer();
            int stride = imageData.Stride;

            for (int y = 0; y < imageData.Height; y++)
            {
                byte* row = scan0 + (y * stride);

                for (int x = 0; x < imageData.Width; x++)
                {
                    // Watch out for actual order (BGR)!
                    int bIndex = x * bytesPerPixel;
                    int gIndex = bIndex + 1;
                    int rIndex = bIndex + 2;

                    byte pixelR = row[rIndex];
                    byte pixelG = row[gIndex];
                    byte pixelB = row[bIndex];
                    if (pixelR>0||pixelG>0||pixelB>0)
                    {
                        result.Add(new System.Windows.Point(x, y));
                    }

                }
            }

            image.UnlockBits(imageData);
            return result;
        }




        private unsafe List<System.Windows.Point> filterColorWithUnsafe(Bitmap image, List<System.Windows.Point> points)
        {
            List<System.Windows.Point> result = new List<System.Windows.Point>();

            BitmapData imageData = image.LockBits(new Rectangle(0, 0, image.Width,
              image.Height), ImageLockMode.ReadWrite, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
            int bytesPerPixel = 3;

            byte* scan0 = (byte*)imageData.Scan0.ToPointer();
            int stride = imageData.Stride;

            for(int i = 0;i<points.Count;i++){
                int x = (int)points[i].X;
                int y = (int)points[i].Y;

                byte* row = scan0 + (y * stride);

                int bIndex = x * bytesPerPixel;
                int gIndex = bIndex + 1;
                int rIndex = bIndex + 2;

                byte pixelR = row[rIndex];
                byte pixelG = row[gIndex];
                byte pixelB = row[bIndex];
                if (pixelR > 0 || pixelG > 0 || pixelB > 0)
                {
                    result.Add(new System.Windows.Point(x, y));
                }
            }

            image.UnlockBits(imageData);
            return result;
        }




        private List<System.Windows.Point> listLightedPixels(BitmapSource bitmapSource)
        {
            List<System.Windows.Point> result = new List<System.Windows.Point>();
            Bitmap myBitmap = this.getBitmap(bitmapSource);
            List<System.Windows.Point> myResult = this.detectColorWithUnsafe(myBitmap);

            //System.Windows.Media.Color pixelCol = this.GetPixelColor(bitmapSource, 0, 0);
            //Console.WriteLine("this.GetPixelColor(bitmapSource, 0,0) {0} {1} {2} {3} {4}", pixelCol,pixelCol.A,pixelCol.R,pixelCol.B,pixelCol.G);

            /*for (int y = 0; y < 1080;y++ )
            {
                for (int x = 0; x < 1920;x++ )
                {
                    System.Windows.Media.Color pixelColor = this.GetPixelColor(bitmapSource, x, y);
                    if(pixelColor.R>0||pixelColor.B>0||pixelColor.G>0){
                        result.Add(new System.Windows.Point(x, y));
                    }
                }
            }*/
            myBitmap.Dispose();
            return myResult;
        }


        private List<System.Windows.Point> filterLightedPixels(BitmapSource bitmapSource, List<System.Windows.Point> inputList)
        {
            List<System.Windows.Point> result = new List<System.Windows.Point>();
            Bitmap myBitmap = this.getBitmap(bitmapSource);
            List<System.Windows.Point> myResult = this.filterColorWithUnsafe(myBitmap, inputList);
            myBitmap.Dispose();
            return myResult;
        }

        private List<List<System.Windows.Point>> filterLightedPixels2(BitmapSource bitmapSource, List<List<System.Windows.Point>> inputList, ProjectablePattern image, List<System.Windows.Point> projector2d)
        {
            List<List<System.Windows.Point>> myResult = new List<List<System.Windows.Point>>();
            Bitmap myBitmap = this.getBitmap(bitmapSource);
            for (int a = 0; a < projector2d.Count; a++)
            {
                if (image.pixelIsWhite(((int)projector2d[a].X), ((int)projector2d[a].Y)) && image.getDistanceOfPixelToBorder(((int)projector2d[a].X), ((int)projector2d[a].Y)) >= 1)
                {        
                    myResult.Add(this.filterColorWithUnsafe(myBitmap, inputList[a]));   
                }
                else {
                    myResult.Add(inputList[a]);
                }
            }
            myBitmap.Dispose();
            
            return myResult;
        }





        public void calculateCorrespondances() {
            Console.WriteLine("CorrespondancesBuilder says:  Bufferlength: {0}, {1}", this.verticalImages.Count, this.horizontalImages.Count);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + "horizontal";
            bool exists = System.IO.Directory.Exists(myPhotos);
            if (!exists) System.IO.Directory.CreateDirectory(myPhotos);
            string path = Path.Combine(myPhotos, "frame" + "0" + ".png");
            BitmapSource bitmapSource1 = this.getBitmapFrame(path);
            Console.WriteLine("bitmapSource: Width: {0}, Height: {1}", bitmapSource1.Width, bitmapSource1.Height);

            int stride1 = bitmapSource1.PixelWidth * 4;
            int size1 = bitmapSource1.PixelHeight * stride1;
            byte[] pixels1 = new byte[size1];
            bitmapSource1.CopyPixels(pixels1, stride1, 0);



            path = Path.Combine(myPhotos, "frame" + "1" + ".png");
            BitmapSource bitmapSource2 = this.getBitmapFrame(path);

            

            int stride2 = bitmapSource2.PixelWidth * 4;
            int size2 = bitmapSource2.PixelHeight * stride1;
            byte[] pixels2 = new byte[size2];
            bitmapSource2.CopyPixels(pixels2, stride2, 0);

            var result = this.convertByteArrayToBitmapSource(bitmapSource1, pixels1);

            this.screenshot(0, "test",result);

            result = this.convertByteArrayToBitmapSource(bitmapSource1, pixels2);
            this.screenshot(1, "test", result);

            result = this.subtractTwoBitmapSources(bitmapSource1, bitmapSource2, 150, -300);
            this.screenshot(2, "test", result);
        }

        private void screenshot(int frameNumber, string subfolder, BitmapSource bitmap)
        {
            BitmapEncoder encoder = new PngBitmapEncoder();
            encoder.Frames.Add(BitmapFrame.Create(bitmap));

            string time = System.DateTime.Now.ToString("hh'-'mm'-'ss", CultureInfo.CurrentUICulture.DateTimeFormat);

            string myPhotos = Environment.GetFolderPath(Environment.SpecialFolder.MyPictures) + "\\projectorCalibration\\kinect" + "\\" + subfolder;
            bool exists = System.IO.Directory.Exists(myPhotos);
            if (!exists) System.IO.Directory.CreateDirectory(myPhotos);

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

    }
}
