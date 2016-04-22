using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using System.Threading;


namespace Microsoft.Samples.Kinect.CoordinateMappingBasics
{
    /// <summary>
    /// Interaction logic for Projector.xaml
    /// </summary>
    public partial class Projector : Window
    {
        public Projector()
        {
            InitializeComponent();
        }

        public void sayHello()
        {
            Console.WriteLine("Hello");
        }
        public void drawLine() {
            var line = new Line();
            line.Stroke = Brushes.LightSteelBlue;

            line.X1 = 0;
            line.Y1 = 0;
            line.X2 = 1920;
            line.Y2 = 1200;

            line.StrokeThickness = 1;
            myCanvas.Children.Add(line);

            var line2 = new Line();
            line2.Stroke = Brushes.LightSteelBlue;

            line2.X1 = 100;
            line2.Y1 = 0;
            line2.X2 = 100;
            line2.Y2 = 10;

            line2.StrokeThickness = 1;
            myCanvas.Children.Add(line2);

            var line3 = new Line();
            line3.Stroke = Brushes.LightSteelBlue;

            line3.X1 = 1278;
            line3.Y1 = 0;
            line3.X2 = 1278;
            line3.Y2 = 960;

            line3.StrokeThickness = 1;
            myCanvas.Children.Add(line3);
        }

        public void addPixel(double x, double y)
        {
            Rectangle rec = new Rectangle();
            Canvas.SetTop(rec, y);
            Canvas.SetLeft(rec, x);
            rec.Width = 1;
            rec.Height = 1;
            rec.Fill = new SolidColorBrush(Colors.White);
            myCanvas.Children.Add(rec);
        }

        public void drawRectangle(double x, double y, double width, double height)
        {
            
            Rectangle rec = new Rectangle();

            Canvas.SetTop(rec, y);
            Canvas.SetLeft(rec, x);
            rec.Width = width;
            rec.Height = height;
            rec.Fill = new SolidColorBrush(Colors.White);
            myCanvas.Children.Add(rec);
        }

        public void clearCanvas() {
            myCanvas.Children.Clear();
        }


        public void changeCanvasSize(int width, int height) {
            myCanvas.Height = height;
            myCanvas.Width = width;
        }
    }
}
