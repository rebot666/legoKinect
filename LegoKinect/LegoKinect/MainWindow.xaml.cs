using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Threading;

using System.Runtime.InteropServices;
//using System.Drawing.Imaging;
using Microsoft.Kinect;
using NxtNet;

namespace LegoKinect
{
    /// <summary>
    /// Lógica de interacción para MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        //Runtime kinect;
        private static readonly int Bgr32BytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        ColorImageFormat lastImageFormat = ColorImageFormat.Undefined;
        WriteableBitmap outputImage;
        WriteableBitmap outputBitmap;
        KinectSensor kinectS;
        private Nxt _nxt;
        Thread Motor;
        byte[] pixelData;
        
        DepthImageFormat lastImageFormat1 = DepthImageFormat.Undefined;
        short[] pixelData1;
        byte[] depthFrame32;
        short[] pixelDataD;
        int RedIndex = 2;
        int GreenIndex = 1;
        int BlueIndex = 0;
        readonly int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
        readonly int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
        readonly int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

        Skeleton[] skeletons;

        //Para la deteccion de las manos
        int HandLeftX = -1, HandLeftY = -1, HandRightX, HandRightY;
        
        //Lego
        bool flagConexion = true;
        bool izq = false;
        bool der = false;
        bool ava = false;
        bool rev = false;
        bool stop = false;

        public MainWindow()
        {
            InitializeComponent();
            Lego_Initialize();
            //Inicializa kinect
                MainWindow_Loaded();
            //Despliega en image1 con color
                //Videoframe ready Initialize
                kinectS.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                //kinectS.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(ColorImageReady);
                kinectS.ColorFrameReady += ColorImageReady;
                

            //DepthFrameReady
                
                kinectS.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);
                kinectS.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(DepthImageReady);
                //kinectS.DepthFrameReady += DepthFrameReady;
            //Skeleton
                //kinectS.SkeletonFrameReady += new
                //EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
                kinectS.SkeletonStream.Enable();
                //kinectS.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
                kinectS.AllFramesReady += FramesReady;
            
            //App
                cIx.Content = HandLeftX;
                cIy.Content = HandLeftY;

                cDx.Content = HandRightX;
                cDy.Content = HandRightY;

            //MainWindow_Closed();
            
            
        }

        void Lego_Initialize() {
            if (flagConexion)
            {
                this._nxt = new Nxt();
                this._nxt.Connect("COM18");
                msjConexion.Text = "Lego Conectado!";
                

                flagConexion = false;
                //Motor = new Thread(new ThreadStart(detenerse));
                //Motor.Start();
                Thread.Sleep(1000);
            }
        }
        void MainWindow_Loaded() { 
            kinectS = KinectSensor.KinectSensors[0];
            try
            {
                kinectS.ColorStream.Enable();
                kinectS.DepthStream.Enable();
                kinectS.SkeletonStream.Enable();
                kinectS.Start();
                //MessageBox.Show("El Kinect esta conectado!!");
                
            }
            catch {
                MessageBox.Show("Error en la conexion!!");
            }

        }

        
        void avanza() {
            this._nxt.SetOutputState(MotorPort.PortA, 75, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
            this._nxt.SetOutputState(MotorPort.PortB, 75, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
        }
        
        void MainWindow_Closed() {
            kinectS.Stop();
            MessageBox.Show("Conexion Cerrada");
            Environment.Exit(0);
        }

        

        private void ColorImageReady(object sender, ColorImageFrameReadyEventArgs e)
        {


            using (ColorImageFrame imageFrame = e.OpenColorImageFrame())
            {
                if (imageFrame != null)
                {
                    // We need to detect if the format has changed.
                    bool haveNewFormat = this.lastImageFormat != imageFrame.Format;

                    if (haveNewFormat)
                    {
                        this.pixelData = new byte[imageFrame.PixelDataLength];
                    }

                    imageFrame.CopyPixelDataTo(this.pixelData);

                    // A WriteableBitmap is a WPF construct that enables resetting the Bits of the image.
                    // This is more efficient than creating a new Bitmap every frame.
                    if (haveNewFormat)
                    {
                        
                        image1.Visibility = Visibility.Visible;
                        this.outputImage = new WriteableBitmap(imageFrame.Width,imageFrame.Height,96, 96, PixelFormats.Bgr32,null);

                        this.image1.Source = this.outputImage;
                    }

                    this.outputImage.WritePixels(new Int32Rect(0, 0, imageFrame.Width, imageFrame.Height),this.pixelData,imageFrame.Width * Bgr32BytesPerPixel,0);

                    this.lastImageFormat = imageFrame.Format;

                    //UpdateFrameRate();
                }
            }
        }

        /*private void DepthImageReady(object sender, DepthImageFrameReadyEventArgs e){
            using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame != null)
                {
                    // We need to detect if the format has changed.
                    bool haveNewFormat = this.lastImageFormat1 != imageFrame.Format;

                    if (haveNewFormat)
                    {
                        this.pixelData1 = new short[imageFrame.PixelDataLength];
                        this.depthFrame32 = new byte[imageFrame.Width * imageFrame.Height * Bgr32BytesPerPixel];
                    }

                    imageFrame.CopyPixelDataTo(this.pixelData1);

                    byte[] convertedDepthBits = this.ConvertDepthFrame(this.pixelData1, ((KinectSensor)sender).DepthStream);

                    // A WriteableBitmap is a WPF construct that enables resetting the Bits of the image.
                    // This is more efficient than creating a new Bitmap every frame.
                    if (haveNewFormat)
                    {
                        this.outputBitmap = new WriteableBitmap(
                            imageFrame.Width,
                            imageFrame.Height,
                            96,  // DpiX
                            96,  // DpiY
                            PixelFormats.Bgr32,
                            null);

                        this.image2.Source = this.outputBitmap;
                    }

                    this.outputBitmap.WritePixels(
                        new Int32Rect(0, 0, imageFrame.Width, imageFrame.Height),
                        convertedDepthBits,
                        imageFrame.Width * Bgr32BytesPerPixel,
                        0);

                    this.lastImageFormat1 = imageFrame.Format;

                    //UpdateFrameRate();
                }
            }
        }
        
        byte[] ConvertDepthFrame(short[] depthFrame, DepthImageStream depthStream)
        {
            int tooNearDepth = depthStream.TooNearDepth;
            int tooFarDepth = depthStream.TooFarDepth;
            int unknownDepth = depthStream.UnknownDepth;
            
            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < this.depthFrame32.Length; i16++, i32 += 4)
            {
                int player = depthFrame[i16] & DepthImageFrame.PlayerIndexBitmask;
                int realDepth = depthFrame[i16] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                // transform 13-bit depth information into an 8-bit intensity appropriate
                // for display (we disregard information in most significant bit)
                byte intensity = (byte)(~(realDepth >> 4));
                
                if (player == 0 && realDepth == 0)
                {
                    // white 
                    this.depthFrame32[i32 + RedIndex] = 255;
                    this.depthFrame32[i32 + GreenIndex] = 255;
                    this.depthFrame32[i32 + BlueIndex] = 255;
                    mensaje.Text = "Blanco";
                }
                else if (player == 0 && realDepth == tooFarDepth)
                {
                    // dark purple
                    this.depthFrame32[i32 + RedIndex] = 66;
                    this.depthFrame32[i32 + GreenIndex] = 0;
                    this.depthFrame32[i32 + BlueIndex] = 66;
                    mensaje.Text = "Morado";
                }
                else if (player == 0 && realDepth == unknownDepth)
                {
                    // dark brown
                    this.depthFrame32[i32 + RedIndex] = 66;
                    this.depthFrame32[i32 + GreenIndex] = 66;
                    this.depthFrame32[i32 + BlueIndex] = 33;
                    mensaje.Text = "Cafe";
                }
                else
                {
                    // tint the intensity by dividing by per-player values
                    this.depthFrame32[i32 + RedIndex] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                    this.depthFrame32[i32 + GreenIndex] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                    this.depthFrame32[i32 + BlueIndex] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                }
            }

            return this.depthFrame32;

        }
         */
        /*
        void DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            DepthImageFrame imageFrame = e.OpenDepthImageFrame();
            
            if (imageFrame != null)
            {
                short[] pixelData = new short[imageFrame.PixelDataLength];
                imageFrame.CopyPixelDataTo(pixelData);
                int[] depth = new int[imageFrame.PixelDataLength];
                int[] player = new int[imageFrame.PixelDataLength];
                int[] playercoded = new int[imageFrame.PixelDataLength];

                for (int i = 0; i < depth.Length; i++)
                {
                    player[i] = pixelData[i] & DepthImageFrame.PlayerIndexBitmask;
                    depth[i] = ((ushort)pixelData[i]) >> DepthImageFrame.PlayerIndexBitmaskWidth;

                    playercoded[i] = 0;
                    
                    if ((player[i] & 0x01) == 0x01)
                    {
                        playercoded[i] |= 0xFF0000;
                    }else
                    if ((player[i] & 0x02) == 0x02)
                    {
                        playercoded[i] |= 0x00FF00;
                    }else
                    if ((player[i] & 0x04) == 0x04)
                    {
                        playercoded[i] |= 0x0000FF;
                    }
                }

                image2.Source = IntToBitmap(depth, imageFrame.Width, imageFrame.Height);
                //image2.Source = IntToBitmap(playercoded, imageFrame.Width, imageFrame.Height);
            }
        }
        */

        private void DepthImageReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame != null)
                {
                    // We need to detect if the format has changed.
                    bool haveNewFormat = imageFrame != null;

                    if (haveNewFormat)
                    {
                        pixelDataD = new short[imageFrame.PixelDataLength];
                        imageFrame.CopyPixelDataTo(pixelDataD);
                        this.depthFrame32 = new byte[imageFrame.Width * imageFrame.Height * Bgr32BytesPerPixel];
                    }

                    
                    imageFrame.CopyPixelDataTo(pixelDataD);

                    byte[] convertedDepthBits = this.ConvertDepthFrame(pixelDataD, ((KinectSensor)sender).DepthStream);

                    // A WriteableBitmap is a WPF construct that enables resetting the Bits of the image.
                    // This is more efficient than creating a new Bitmap every frame.
                    if (haveNewFormat)
                    {
                        this.outputBitmap = new WriteableBitmap(
                            imageFrame.Width,
                            imageFrame.Height,
                            96,  // DpiX
                            96,  // DpiY
                            PixelFormats.Bgr32,
                            null);

                        image2.Source = this.outputBitmap;
                        
                        
                    }

                    this.outputBitmap.WritePixels(
                        new Int32Rect(0, 0, imageFrame.Width, imageFrame.Height),
                        convertedDepthBits,
                        imageFrame.Width * Bgr32BytesPerPixel,
                        0);

                    //this.lastImageFormat = imageFrame.Format;

                    //UpdateFrameRate();
                }
            }
        }

        private byte[] ConvertDepthFrame(short[] depthFrame, DepthImageStream depthStream)
        {
            int tooNearDepth = depthStream.TooNearDepth;
            int tooFarDepth = depthStream.TooFarDepth;
            int unknownDepth = depthStream.UnknownDepth;

            for (int i16 = 0, i32 = 0; i16 < depthFrame.Length && i32 < this.depthFrame32.Length; i16++, i32 += 4)
            {
                int player = depthFrame[i16] & DepthImageFrame.PlayerIndexBitmask;
                int realDepth = depthFrame[i16] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                // transform 13-bit depth information into an 8-bit intensity appropriate
                // for display (we disregard information in most significant bit)
                byte intensity = (byte)(~(realDepth >> 4));
                
                if (player == 0 && realDepth == 0)
                {
                    // white 
                    this.depthFrame32[i32 + RedIndex] = 255;
                    this.depthFrame32[i32 + GreenIndex] = 255;
                    this.depthFrame32[i32 + BlueIndex] = 255;
                    mensaje.Text = "Blanco";
                }
                else if (player == 0 && realDepth == tooFarDepth)
                {
                    // dark purple
                    this.depthFrame32[i32 + RedIndex] = 66;
                    this.depthFrame32[i32 + GreenIndex] = 0;
                    this.depthFrame32[i32 + BlueIndex] = 66;
                    mensaje.Text = "Morado";
                }
                else if (player == 0 && realDepth == unknownDepth)
                {
                    // dark brown
                    this.depthFrame32[i32 + RedIndex] = 66;
                    this.depthFrame32[i32 + GreenIndex] = 66;
                    this.depthFrame32[i32 + BlueIndex] = 33;
                    mensaje.Text = "Cafe";
                }
                else
                {
                    // tint the intensity by dividing by per-player values
                    this.depthFrame32[i32 + RedIndex] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                    this.depthFrame32[i32 + GreenIndex] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                    this.depthFrame32[i32 + BlueIndex] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                }
            }

            return this.depthFrame32;
        }
        BitmapSource IntToBitmap(int[] array, int w, int h)
        {
            BitmapSource bmap = BitmapSource.Create(w,h,96, 96,PixelFormats.Bgr32,null,array,w * 4);
            return bmap;
        }

        BitmapSource DepthToBitmapSource(DepthImageFrame imageFrame){
            short[] pixelData = new short[imageFrame.PixelDataLength];
            imageFrame.CopyPixelDataTo(pixelData);

            BitmapSource bmap = BitmapSource.Create(imageFrame.Width, imageFrame.Height,96, 96,PixelFormats.Gray16,null,pixelData,imageFrame.Width * imageFrame.BytesPerPixel);
            return bmap;
        }

        private void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e){
            bool receivedData = false;
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    if (skeletons == null) //allocate the first time
                    {
                        skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    }
                    receivedData = true;
                }
                else
                {
                    // apps processing of skeleton data took too long; it got more than 2 frames behind.
                    // thedata is no longer avabilable.
                }
            }
            if (receivedData)
            {
                
                // DISPLAYOR PROCESS IMAGE DATA IN skeletons HERE
            }
        }

        Point getJoint(JointType j, Skeleton S)
        {
            SkeletonPoint Sloc = S.Joints[j].Position;
            ColorImagePoint Cloc = kinectS.MapSkeletonPointToColor(Sloc, ColorImageFormat.RgbResolution640x480Fps30);

            return new Point(Cloc.X, Cloc.Y);
        }

        void FramesReady(object sender, AllFramesReadyEventArgs e)
        {
            ColorImageFrame VFrame = e.OpenColorImageFrame();
            if (VFrame == null) return;
            byte[] pixelS = new byte[VFrame.PixelDataLength];
            //BitmapSource bmap = ImageToBitmap(VFrame);
            Point p1, p2;
            SkeletonFrame SFrame = e.OpenSkeletonFrame();
            if (SFrame == null) return;

            Skeleton[] Skeletons = new Skeleton[SFrame.SkeletonArrayLength];
            SFrame.CopySkeletonDataTo(Skeletons);

            foreach (Skeleton S in Skeletons)
            {
                if (S.TrackingState == SkeletonTrackingState.Tracked)
                {
                    p1 = getJoint(JointType.HandLeft,S);
                    
                    HandLeftX = (int)p1.X;
                    HandLeftY = (int)p1.Y;
                    cIx.Content = HandLeftX;
                    cIy.Content = HandLeftY;

                    p2 = getJoint(JointType.HandRight, S);
                    
                    HandRightX = (int)p2.X;
                    HandRightY = (int)p2.Y;
                    cDx.Content = HandRightX;
                    cDy.Content = HandRightY;

                    verificarLego();
                }
            }
            
        }

        //Metodos del Lego
        public void avanzar()
        {
            this._nxt.SetOutputState(MotorPort.PortA, 75, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
            this._nxt.SetOutputState(MotorPort.PortB, 75, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);

        }


        public void detenerse()
        {
            this._nxt.PlayTone(601, 1000);
            this._nxt.SetOutputState(MotorPort.PortA, 0, MotorModes.Brake, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
            this._nxt.SetOutputState(MotorPort.PortB, 0, MotorModes.Brake, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
        }

        public void reversa()
        {
            this._nxt.SetOutputState(MotorPort.PortA, -75, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
            this._nxt.SetOutputState(MotorPort.PortB, -75, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
        }

        public void derecha()
        {

            this._nxt.SetOutputState(MotorPort.PortA, 80, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
            this._nxt.SetOutputState(MotorPort.PortB, -80, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
        }

        public void izquierda()
        {
            this._nxt.SetOutputState(MotorPort.PortA, -80, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
            this._nxt.SetOutputState(MotorPort.PortB, 80, MotorModes.On, MotorRegulationMode.Sync, 0, MotorRunState.Running, 0);
        }

        public void verificarLego()
        {
            if (HandRightX >= 400 && HandRightY >= 100 && HandRightY <= 300 && !der && HandLeftX >= 180)
            {
                Motor = new Thread(new ThreadStart(derecha));
                Motor.Start();
                der = true;
                izq = false;
                ava = false;
                rev = false;
                stop = false;
                msjLego.Text = "Derecha!";
            }

            else if (HandLeftX <= 130 && HandLeftY >= 100 && HandLeftY <= 300 && !izq && HandRightX < 400)
            {
                Motor = new Thread(new ThreadStart(izquierda));
                Motor.Start();
                der = false;
                izq = true;
                ava = false;
                rev = false;
                stop = false;
                msjLego.Text = "Izquierda!";
            }

            else if (HandRightY <= 100 && HandLeftY <= 100 && !ava /*&& HandRightX >= 300 && HandLeftX >= 180 && HandLeftX >= 200*/ )
            //else if (HandRightY <= 300 && HandLeftY <= 300 && !ava && HandRightX >= 300 && HandLeftX <= 300  /*&& HandRightX >= 300 && HandLeftX >= 180 && HandLeftX >= 200*/ )
            {
                Motor = new Thread(new ThreadStart(avanzar));
                Motor.Start();
                der = false;
                izq = false;
                ava = true;
                rev = false;
                stop = false;
                msjLego.Text = "Avanzar!";
            }

            else if (HandRightY <= 300 && HandLeftY <= 300 && !rev && HandLeftY >= 100 && HandRightY >= 100 && HandRightX < 400 && HandRightX > 300 && HandLeftX > 130 && HandLeftX < 200)
            {
                Motor = new Thread(new ThreadStart(reversa));
                Motor.Start();
                der = false;
                izq = false;
                ava = false;
                rev = true;
                stop = false;
                msjLego.Text = "Reversa!";
            }

            else if (HandRightY >= 330 && HandLeftY >= 330 && !stop)
            {
                Motor = new Thread(new ThreadStart(detenerse));
                Motor.Start();
                der = false;
                izq = false;
                ava = false;
                rev = false;
                stop = true;
                msjLego.Text = "Stop!";
            }


        }
    }
}