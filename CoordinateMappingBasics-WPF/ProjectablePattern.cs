using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

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

namespace Microsoft.Samples.Kinect.CoordinateMappingBasics
{
    public class ProjectablePattern
    {
        private int heightResolution;
        private int widthResolution;
        private bool isHeightPattern;//false = Vertical pattern, true = Horizontal pattern
        private int offset;
        private int stride;//length of projected bars
        private Boolean whiteFirst; //determines whether to project white or black first
        private int iteration;
        private Microsoft.Samples.Kinect.CoordinateMappingBasics.Projector myProjector = null;


        public ProjectablePattern(int widthResolution, int heightResolution, bool isHeightPattern, int iteration, Microsoft.Samples.Kinect.CoordinateMappingBasics.Projector myProjector)
        {
            this.heightResolution = heightResolution;
            this.widthResolution = widthResolution;
            this.isHeightPattern = isHeightPattern;
            this.iteration = iteration;
            this.myProjector = myProjector;
        }

        private bool calculateOffsetAndStride() {
            int resolution = 0;
            bool success = false;
            if (isHeightPattern)
            {
                resolution = this.heightResolution;
            }
            else {
                resolution = this.widthResolution;
            }

            switch(resolution)
            {
                case 768:
                    success = this.resolution768();
                    break;
                case 480:
                    success = this.resolution480();
                    break;
                case 640:
                    success = this.resolution640();
                    break;
                case 1024:
                    success = this.resolution1024();
                    break;
                case 1280:
                    success = this.resolution1280();
                    break;
                case 800:
                    success = this.resolution800();
                    break;
                default:
                    Console.WriteLine("No resolution found!!!");
                    break;
            }

            return success;

        }

        public bool pixelIsWhite(int x, int y)
        {
            int myValue = 0;
            if (this.isHeightPattern)
            {
                myValue = y;
            }
            else {
                myValue = x;
            }

            if(this.whiteFirst){
                if ((((myValue - this.offset) / this.stride) % 2) == 0)
                {
                    return true;
                }
                else {
                    return false;
                }
            }
            else
            {
                if ((((myValue - this.offset) / this.stride) % 2) == 1)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }

        }


        public int getDistanceOfPixelToBorder(int x, int y) {

            int myValue = 0;
            if (this.isHeightPattern)
            {
                myValue = y;
            }
            else
            {
                myValue = x;
            }
           
            int result = (myValue - this.offset) % stride;
            return Math.Min(this.stride-result,result-1);
        }




        public bool projectPattern() {
            bool success = this.calculateOffsetAndStride();
            if (!success)
            {
                return false;
            }

            this.myProjector.clearCanvas();
            
            if (isHeightPattern)
            {
                for (int i = 0; i < (this.heightResolution / this.stride); i++)
                {
                    if (this.whiteFirst)
                    {
                        if(i % 2 == 0){                      
                            this.myProjector.drawRectangle(0, this.stride * i + this.offset, this.widthResolution, this.stride);
                        }
                        
                    }
                    else {
                        if (i % 2 == 1)
                        {
                            this.myProjector.drawRectangle(0, this.stride * i + this.offset, this.widthResolution, this.stride);
                        }
                    }
                }
            }
            else {
                for (int i = 0; i < (this.widthResolution / this.stride); i++)
                {
                    if (this.whiteFirst)
                    {
                        if (i % 2 == 0)
                        {
                            this.myProjector.drawRectangle(this.stride * i + this.offset, 0, this.stride, this.heightResolution);
                        }

                    }
                    else
                    {
                        if (i % 2 == 1)
                        {
                            this.myProjector.drawRectangle(this.stride * i + this.offset, 0, this.stride, this.heightResolution);
                        }
                    }
                }
            }
            return true;
        }

        public int[] getContraryPhotos(int resolution, int iteration) {

            switch (resolution) { 
                case 1280:
                    break;
                        

                default:
                    Console.WriteLine("getPairingPhotos in PP says: No such resolution");
                    return null;
            }

            return null;
        }





        //actual resolution helpers

        private int[] contraryPhotos1280(int iteration)
        {

            if (iteration <= 14)
            {

                if (iteration % 2 == 0)
                {
                    return new int[] { iteration, iteration - 1 };
                }
                else
                {
                    return new int[] { iteration, iteration + 1 };
                }
            }
            else if(iteration<=25){ 
            
            }

            return null;
        }


        private bool resolution1280()
        {
            switch (this.iteration)
            {
                case 1:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 640;
                    return true;
                case 2:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 640;
                    return true;
                case 3:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 320;
                    return true;
                case 4:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 320;
                    return true;
                case 5:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 160;
                    return true;
                case 6:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 160;
                    return true;
                case 7:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 80;
                    return true;
                case 8:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 80;
                    return true;
                case 9:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 40;
                    return true;
                case 10:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 40;
                    return true;
                case 11:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 12:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 13:
                    this.whiteFirst = true;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 14:
                    this.whiteFirst = false;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 15:
                    this.whiteFirst = true;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 16:
                    this.whiteFirst = false;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 17:
                    this.whiteFirst = true;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 18:
                    this.whiteFirst = false;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 19:
                    this.whiteFirst = true;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 20:
                    this.whiteFirst = false;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 21:
                    this.whiteFirst = true;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 22:
                    this.whiteFirst = false;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 23:
                    this.whiteFirst = true;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 24:
                    this.whiteFirst = false;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 25:
                    this.whiteFirst = true;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 26:
                    this.whiteFirst = false;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 27:
                    this.whiteFirst = true;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 28:
                    this.whiteFirst = false;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 29:
                    this.whiteFirst = true;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 30:
                    this.whiteFirst = false;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 31:
                    this.whiteFirst = true;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 32:
                    this.whiteFirst = false;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 33:
                    this.whiteFirst = true;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 34:
                    this.whiteFirst = false;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 35:
                    this.whiteFirst = true;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 36:
                    this.whiteFirst = false;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 37:
                    this.whiteFirst = true;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 38:
                    this.whiteFirst = false;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 39:
                    this.whiteFirst = true;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 40:
                    this.whiteFirst = false;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 41:
                    this.whiteFirst = true;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 42:
                    this.whiteFirst = false;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 43:
                    this.whiteFirst = true;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 44:
                    this.whiteFirst = false;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 45:
                    this.whiteFirst = true;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 46:
                    this.whiteFirst = false;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 47:
                    this.whiteFirst = true;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 48:
                    this.whiteFirst = false;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 49:
                    this.whiteFirst = true;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 50:
                    this.whiteFirst = false;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 51:
                    this.whiteFirst = true;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 52:
                    this.whiteFirst = false;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 53:
                    this.whiteFirst = true;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 54:
                    this.whiteFirst = false;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 55:
                    this.whiteFirst = true;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 56:
                    this.whiteFirst = false;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 57:
                    this.whiteFirst = true;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 58:
                    this.whiteFirst = false;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 59:
                    this.whiteFirst = true;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                case 60:
                    this.whiteFirst = false;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                default:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 1024;

                    Console.WriteLine("Default case");
                    return false;
            }
        }

        private bool resolution640()
        {
            switch (this.iteration)
            {
                case 1:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 320;
                    return true;
                case 2:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 320;
                    return true;
                case 3:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 160;
                    return true;
                case 4:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 160;
                    return true;
                case 5:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 80;
                    return true;
                case 6:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 80;
                    return true;
                case 7:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 40;
                    return true;
                case 8:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 40;
                    return true;
                case 9:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 10:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 11:
                    this.whiteFirst = true;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 12:
                    this.whiteFirst = false;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 13:
                    this.whiteFirst = true;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 14:
                    this.whiteFirst = false;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 15:
                    this.whiteFirst = true;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 16:
                    this.whiteFirst = false;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 17:
                    this.whiteFirst = true;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 18:
                    this.whiteFirst = false;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 19:
                    this.whiteFirst = true;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 20:
                    this.whiteFirst = false;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 21:
                    this.whiteFirst = true;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 22:
                    this.whiteFirst = false;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 23:
                    this.whiteFirst = true;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 24:
                    this.whiteFirst = false;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 25:
                    this.whiteFirst = true;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 26:
                    this.whiteFirst = false;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 27:
                    this.whiteFirst = true;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 28:
                    this.whiteFirst = false;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 29:
                    this.whiteFirst = true;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 30:
                    this.whiteFirst = false;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 31:
                    this.whiteFirst = true;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 32:
                    this.whiteFirst = false;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 33:
                    this.whiteFirst = true;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 34:
                    this.whiteFirst = false;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 35:
                    this.whiteFirst = true;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 36:
                    this.whiteFirst = false;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 37:
                    this.whiteFirst = true;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 38:
                    this.whiteFirst = false;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 39:
                    this.whiteFirst = true;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 40:
                    this.whiteFirst = false;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 41:
                    this.whiteFirst = true;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 42:
                    this.whiteFirst = false;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 43:
                    this.whiteFirst = true;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 44:
                    this.whiteFirst = false;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 45:
                    this.whiteFirst = true;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 46:
                    this.whiteFirst = false;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 47:
                    this.whiteFirst = true;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 48:
                    this.whiteFirst = false;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 49:
                    this.whiteFirst = true;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 50:
                    this.whiteFirst = false;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 51:
                    this.whiteFirst = true;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 52:
                    this.whiteFirst = false;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 53:
                    this.whiteFirst = true;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 54:
                    this.whiteFirst = false;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 55:
                    this.whiteFirst = true;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 56:
                    this.whiteFirst = false;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 57:
                    this.whiteFirst = true;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                case 58:
                    this.whiteFirst = false;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                default:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 640;

                    Console.WriteLine("Default case");
                    return false;
            }
        }

        private bool resolution480()
        {
            switch (this.iteration)
            {
                case 1:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 240;
                    return true;
                case 2:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 240;
                    return true;
                case 3:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 120;
                    return true;
                case 4:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 120;
                    return true;
                case 5:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 60;
                    return true;
                case 6:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 60;
                    return true;
                case 7:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 30;
                    return true;
                case 8:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 30;
                    return true;
                case 9:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 10:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 11:
                    this.whiteFirst = true;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 12:
                    this.whiteFirst = false;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 13:
                    this.whiteFirst = true;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 14:
                    this.whiteFirst = false;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 15:
                    this.whiteFirst = true;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 16:
                    this.whiteFirst = false;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 17:
                    this.whiteFirst = true;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 18:
                    this.whiteFirst = false;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 19:
                    this.whiteFirst = true;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 20:
                    this.whiteFirst = false;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 21:
                    this.whiteFirst = true;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 22:
                    this.whiteFirst = false;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 23:
                    this.whiteFirst = true;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 24:
                    this.whiteFirst = false;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 25:
                    this.whiteFirst = true;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 26:
                    this.whiteFirst = false;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 27:
                    this.whiteFirst = true;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 28:
                    this.whiteFirst = false;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 29:
                    this.whiteFirst = true;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 30:
                    this.whiteFirst = false;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 31:
                    this.whiteFirst = true;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 32:
                    this.whiteFirst = false;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 33:
                    this.whiteFirst = true;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 34:
                    this.whiteFirst = false;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 35:
                    this.whiteFirst = true;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 36:
                    this.whiteFirst = false;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 37:
                    this.whiteFirst = true;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 38:
                    this.whiteFirst = false;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 39:
                    this.whiteFirst = true;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 40:
                    this.whiteFirst = false;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 41:
                    this.whiteFirst = true;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 42:
                    this.whiteFirst = false;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 43:
                    this.whiteFirst = true;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 44:
                    this.whiteFirst = false;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 45:
                    this.whiteFirst = true;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 46:
                    this.whiteFirst = false;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 47:
                    this.whiteFirst = true;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 48:
                    this.whiteFirst = false;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 49:
                    this.whiteFirst = true;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 50:
                    this.whiteFirst = false;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 51:
                    this.whiteFirst = true;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 52:
                    this.whiteFirst = false;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 53:
                    this.whiteFirst = true;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 54:
                    this.whiteFirst = false;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 55:
                    this.whiteFirst = true;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 56:
                    this.whiteFirst = false;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 57:
                    this.whiteFirst = true;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                case 58:
                    this.whiteFirst = false;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                default:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 480;

                    Console.WriteLine("Default case");
                    return false;
            }
        }


        private bool resolution1024() {
            switch (this.iteration)
            {
                case 1:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 512;
                    return true;
                case 2:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 512;
                    return true;
                case 3:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 254;
                    return true;
                case 4:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 254;
                    return true;
                case 5:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 128;
                    return true;
                case 6:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 128;
                    return true;
                case 7:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 64;
                    return true;
                case 8:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 64;
                    return true;
                case 9:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 32;
                    return true;
                case 10:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 32;
                    return true;
                case 11:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 16;
                    return true;
                case 12:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 16;
                    return true;
                case 13:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 14:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 15:
                    this.whiteFirst = true;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 16:
                    this.whiteFirst = false;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 17:
                    this.whiteFirst = true;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 18:
                    this.whiteFirst = false;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 19:
                    this.whiteFirst = true;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 20:
                    this.whiteFirst = false;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 21:
                    this.whiteFirst = true;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 22:
                    this.whiteFirst = false;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 23:
                    this.whiteFirst = true;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 24:
                    this.whiteFirst = false;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 25:
                    this.whiteFirst = true;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 26:
                    this.whiteFirst = false;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 27:
                    this.whiteFirst = true;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 28:
                    this.whiteFirst = false;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 29:
                    this.whiteFirst = true;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 30:
                    this.whiteFirst = false;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 31:
                    this.whiteFirst = true;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 32:
                    this.whiteFirst = false;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 33:
                    this.whiteFirst = true;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 34:
                    this.whiteFirst = false;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 35:
                    this.whiteFirst = true;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 36:
                    this.whiteFirst = false;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 37:
                    this.whiteFirst = true;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 38:
                    this.whiteFirst = false;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 39:
                    this.whiteFirst = true;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 40:
                    this.whiteFirst = false;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 41:
                    this.whiteFirst = true;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 42:
                    this.whiteFirst = false;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 43:
                    this.whiteFirst = true;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 44:
                    this.whiteFirst = false;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 45:
                    this.whiteFirst = true;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 46:
                    this.whiteFirst = false;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 47:
                    this.whiteFirst = true;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 48:
                    this.whiteFirst = false;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 49:
                    this.whiteFirst = true;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 50:
                    this.whiteFirst = false;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 51:
                    this.whiteFirst = true;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 52:
                    this.whiteFirst = false;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 53:
                    this.whiteFirst = true;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 54:
                    this.whiteFirst = false;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 55:
                    this.whiteFirst = true;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 56:
                    this.whiteFirst = false;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 57:
                    this.whiteFirst = true;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 58:
                    this.whiteFirst = false;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 59:
                    this.whiteFirst = true;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 60:
                    this.whiteFirst = false;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 61:
                    this.whiteFirst = true;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                case 62:
                    this.whiteFirst = false;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                default:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 1024;

                    Console.WriteLine("Default case");
                    return false;
            }
        }


        private bool resolution800()
        {
            switch (this.iteration)
            {
                case 1:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 400;
                    return true;
                case 2:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 400;
                    return true;
                case 3:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 200;
                    return true;
                case 4:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 200;
                    return true;
                case 5:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 100;
                    return true;
                case 6:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 100;
                    return true;
                case 7:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 50;
                    return true;
                case 8:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 50;
                    return true;
                case 9:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 10:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 11:
                    this.whiteFirst = true;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 12:
                    this.whiteFirst = false;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 13:
                    this.whiteFirst = true;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 14:
                    this.whiteFirst = false;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 15:
                    this.whiteFirst = true;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 16:
                    this.whiteFirst = false;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 17:
                    this.whiteFirst = true;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 18:
                    this.whiteFirst = false;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 19:
                    this.whiteFirst = true;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 20:
                    this.whiteFirst = false;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 21:
                    this.whiteFirst = true;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 22:
                    this.whiteFirst = false;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 23:
                    this.whiteFirst = true;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 24:
                    this.whiteFirst = false;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 25:
                    this.whiteFirst = true;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 26:
                    this.whiteFirst = false;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 27:
                    this.whiteFirst = true;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 28:
                    this.whiteFirst = false;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 29:
                    this.whiteFirst = true;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 30:
                    this.whiteFirst = false;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 31:
                    this.whiteFirst = true;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 32:
                    this.whiteFirst = false;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 33:
                    this.whiteFirst = true;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 34:
                    this.whiteFirst = false;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 35:
                    this.whiteFirst = true;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 36:
                    this.whiteFirst = false;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 37:
                    this.whiteFirst = true;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 38:
                    this.whiteFirst = false;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 39:
                    this.whiteFirst = true;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 40:
                    this.whiteFirst = false;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 41:
                    this.whiteFirst = true;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 42:
                    this.whiteFirst = false;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 43:
                    this.whiteFirst = true;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 44:
                    this.whiteFirst = false;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 45:
                    this.whiteFirst = true;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 46:
                    this.whiteFirst = false;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 47:
                    this.whiteFirst = true;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 48:
                    this.whiteFirst = false;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 49:
                    this.whiteFirst = true;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 50:
                    this.whiteFirst = false;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 51:
                    this.whiteFirst = true;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 52:
                    this.whiteFirst = false;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 53:
                    this.whiteFirst = true;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 54:
                    this.whiteFirst = false;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 55:
                    this.whiteFirst = true;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 56:
                    this.whiteFirst = false;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 57:
                    this.whiteFirst = true;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                case 58:
                    this.whiteFirst = false;
                    this.offset = 24;
                    this.stride = 25;
                    return true;

                default:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 768;

                    Console.WriteLine("Default case");
                    return false;
            }
        }


        private bool resolution768() {
            switch (this.iteration)
            {
                case 1:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 384;
                    return true;
                case 2:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 384;
                    return true;
                case 3:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 192;
                    return true;
                case 4:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 192;
                    return true;
                case 5:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 96;
                    return true;
                case 6:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 96;
                    return true;
                case 7:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 48;
                    return true;
                case 8:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 48;
                    return true;
                case 9:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 10:
                    this.whiteFirst = false;
                    this.offset = 0;
                    this.stride = 25;
                    return true;
                case 11:
                    this.whiteFirst = true;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 12:
                    this.whiteFirst = false;
                    this.offset = 1;
                    this.stride = 25;
                    return true;
                case 13:
                    this.whiteFirst = true;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 14:
                    this.whiteFirst = false;
                    this.offset = 2;
                    this.stride = 25;
                    return true;
                case 15:
                    this.whiteFirst = true;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 16:
                    this.whiteFirst = false;
                    this.offset = 3;
                    this.stride = 25;
                    return true;
                case 17:
                    this.whiteFirst = true;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 18:
                    this.whiteFirst = false;
                    this.offset = 4;
                    this.stride = 25;
                    return true;
                case 19:
                    this.whiteFirst = true;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 20:
                    this.whiteFirst = false;
                    this.offset = 5;
                    this.stride = 25;
                    return true;
                case 21:
                    this.whiteFirst = true;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 22:
                    this.whiteFirst = false;
                    this.offset = 6;
                    this.stride = 25;
                    return true;
                case 23:
                    this.whiteFirst = true;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 24:
                    this.whiteFirst = false;
                    this.offset = 7;
                    this.stride = 25;
                    return true;
                case 25:
                    this.whiteFirst = true;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 26:
                    this.whiteFirst = false;
                    this.offset = 8;
                    this.stride = 25;
                    return true;
                case 27:
                    this.whiteFirst = true;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 28:
                    this.whiteFirst = false;
                    this.offset = 9;
                    this.stride = 25;
                    return true;
                case 29:
                    this.whiteFirst = true;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 30:
                    this.whiteFirst = false;
                    this.offset = 10;
                    this.stride = 25;
                    return true;
                case 31:
                    this.whiteFirst = true;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 32:
                    this.whiteFirst = false;
                    this.offset = 11;
                    this.stride = 25;
                    return true;
                case 33:
                    this.whiteFirst = true;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 34:
                    this.whiteFirst = false;
                    this.offset = 12;
                    this.stride = 25;
                    return true;
                case 35:
                    this.whiteFirst = true;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 36:
                    this.whiteFirst = false;
                    this.offset = 13;
                    this.stride = 25;
                    return true;
                case 37:
                    this.whiteFirst = true;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 38:
                    this.whiteFirst = false;
                    this.offset = 14;
                    this.stride = 25;
                    return true;
                case 39:
                    this.whiteFirst = true;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 40:
                    this.whiteFirst = false;
                    this.offset = 15;
                    this.stride = 25;
                    return true;
                case 41:
                    this.whiteFirst = true;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 42:
                    this.whiteFirst = false;
                    this.offset = 16;
                    this.stride = 25;
                    return true;
                case 43:
                    this.whiteFirst = true;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 44:
                    this.whiteFirst = false;
                    this.offset = 17;
                    this.stride = 25;
                    return true;
                case 45:
                    this.whiteFirst = true;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 46:
                    this.whiteFirst = false;
                    this.offset = 18;
                    this.stride = 25;
                    return true;
                case 47:
                    this.whiteFirst = true;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 48:
                    this.whiteFirst = false;
                    this.offset = 19;
                    this.stride = 25;
                    return true;
                case 49:
                    this.whiteFirst = true;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 50:
                    this.whiteFirst = false;
                    this.offset = 20;
                    this.stride = 25;
                    return true;
                case 51:
                    this.whiteFirst = true;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 52:
                    this.whiteFirst = false;
                    this.offset = 21;
                    this.stride = 25;
                    return true;
                case 53:
                    this.whiteFirst = true;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 54:
                    this.whiteFirst = false;
                    this.offset = 22;
                    this.stride = 25;
                    return true;
                case 55:
                    this.whiteFirst = true;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 56:
                    this.whiteFirst = false;
                    this.offset = 23;
                    this.stride = 25;
                    return true;
                case 57:
                    this.whiteFirst = true;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                case 58:
                    this.whiteFirst = false;
                    this.offset = 24;
                    this.stride = 25;
                    return true;
                default:
                    this.whiteFirst = true;
                    this.offset = 0;
                    this.stride = 768;

                    Console.WriteLine("Default case");
                    return false;
            }
        }

    }
}
