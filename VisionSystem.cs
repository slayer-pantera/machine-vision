using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using HalconDotNet;
using GxIAPINET;

namespace 视觉系统1
{

    public class VisionSystem
    {
        /*
         * handCameraPath为下照相机相关文件的存放路径，比如标定时的机器人姿态，格式为
         * @"D:\机器人\C Sharp\project\总控程序调试\使用NIMS与类封装\使用NIMS与类封装2\bin\Debug\HandCamera\"  
         */
        private string _handCameraPath;
        public string HandCameraPath
        {
            get { return _handCameraPath; }
            set { _handCameraPath = value; }
        }
        /*
         * eyeCameraPath为手部相机相关文件的存放路径，比如标定时的机器人姿态，格式为
         * @"D:\机器人\C Sharp\project\总控程序调试\使用NIMS与类封装\使用NIMS与类封装2\bin\Debug\EyeCamera\"
         */
        private string _eyeCameraPath;
        public string EyeCameraPath
        {
            get { return _eyeCameraPath; }
            set { _eyeCameraPath = value; }
        }
        //deviceNumber为相机的编号
        private string _deviceNumber;
        public string DeviceNumber
        {
            get { return _deviceNumber; }
            set { _deviceNumber = value; }
        }
        //hWindowControl为Halcon的控件，包含一个显示图像处理结果的Halcon图像窗口
        private HWindowControl _windowControl;
        public HWindowControl WindowControl
        {
            get { return _windowControl; }
            set { _windowControl = value; }
        }
        //构造函数
        public VisionSystem(string handCameraPath, string eyeCameraPath, string deviceNumber, HWindowControl hWindowControl)
        {
            this.HandCameraPath = handCameraPath;
            this.EyeCameraPath = eyeCameraPath;
            this.DeviceNumber = deviceNumber;
            this.WindowControl = hWindowControl;
        }
        /// <summary>
        /// 1. Mark点检测
        /// </summary>
        /// <param name="markPosition"></param>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0101下照相机无法获得图像
         *          0102根据面积选择连通区域的个数不为1
         */
        public bool MarkPosition(out double[] markPosition, out int errorCode)
        {
            HObject ho_Image;
            HObject ho_ImageWorld;
            HObject ho_Red, ho_Green, ho_Blue;
            HObject ho_ImageMean;
            HObject ho_ROI;
            HObject ho_ImageROI;
            HObject ho_Region, ho_ConnectedRegions;
            HObject ho_SelectedRegions;
            HObject ho_Circle;


            HTuple hv_Device = "";
            HTuple hv_AcqHandle;
            //HTuple hv_FileName = "D:/视觉系统/实际检测图片/qita/2016-07-18_11_18_09_567.bmp";
            //HTuple hv_CamParFile = "D:/视觉系统/shiyanchengxu/daheng/CameraParameters_down.dat";
            //HTuple hv_PoseFile1 = "D:/视觉系统/shiyanchengxu/daheng/CameraPoseNew_down.dat";
            //HTuple hv_PoseFile2 = "D:/视觉系统/shiyanchengxu/daheng/ObjInToolPose.dat";
            string camParFile = "CameraParameters_down.dat";
            string poseFile1 = "CameraPoseNew_down.dat";
            string poseFile2 = "Obj1InToolPose.dat";
            HTuple hv_CameraParameters;
            HTuple hv_CameraPoseNew1;
            HTuple hv_CameraPoseNew2;
            HTuple hv_Row1, hv_Column1, hv_Row2, hv_Column2;
            HTuple hv_UsedThreshold;
            HTuple hv_Number;
            HTuple hv_Row3, hv_Column3, hv_Radius;
            HTuple hv_x1, hv_y1;
            HTuple hv_ObjInToolPose;
            HTuple hv_HomMat3D;
            HTuple hv_Qx, hv_Qy, hv_Qz;
            HTuple hv_HeightWin;
            HTuple hv_WidthWin;

            if (this.DeviceNumber == "1")
            {
                hv_Device = "2";
            }
            else if (this.DeviceNumber == "2")
            {
                hv_Device = "1";
            }
            try
            {
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", hv_Device, 1, -1, out hv_AcqHandle);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc2");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.41);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.38);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);


                //HOperatorSet.GetImageSize(ho_Image, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
                //    HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
                //    HOperatorSet.DispObj(ho_Image, WindowControl.HalconWindow);

                //HOperatorSet.ReadImage(out ho_Image, hv_FileName);
            }
            catch
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                markPosition = buf;
                errorCode = 0101;
                return false;
            }
            HOperatorSet.ReadCamPar(this.HandCameraPath + camParFile, out hv_CameraParameters);
            HOperatorSet.ReadPose(this.HandCameraPath + poseFile1, out hv_CameraPoseNew1);
            HOperatorSet.SetOriginPose(hv_CameraPoseNew1, -0.025, -0.015, 0, out hv_CameraPoseNew2);
            HOperatorSet.ImageToWorldPlane(ho_Image, out ho_ImageWorld, hv_CameraParameters, hv_CameraPoseNew2, 2000, 1500, 0.000025, "bilinear");
            HOperatorSet.Decompose3(ho_ImageWorld, out ho_Red, out ho_Green, out ho_Blue);
            HOperatorSet.MeanImage(ho_Green, out ho_ImageMean, 5, 5);
            //HOperatorSet.SmoothImage(ho_ImageMean, out ho_ImageSmooth, "deriche2", 10);
            hv_Row1 = 215;
            hv_Column1 = 1045;
            hv_Row2 = 507;
            hv_Column2 = 1400;
            HOperatorSet.GenRectangle1(out ho_ROI, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            HOperatorSet.ReduceDomain(ho_ImageMean, ho_ROI, out ho_ImageROI);
            HOperatorSet.BinaryThreshold(ho_ImageROI, out ho_Region, "max_separability", "light", out hv_UsedThreshold);
            HOperatorSet.Connection(ho_Region, out ho_ConnectedRegions);
            HOperatorSet.SelectShape(ho_ConnectedRegions, out ho_SelectedRegions, "area", "and", 500, 3000);
            HOperatorSet.SelectShape(ho_SelectedRegions, out ho_SelectedRegions, "circularity", "and", 0.6, 1);
            HOperatorSet.CountObj(ho_SelectedRegions, out hv_Number);
            HOperatorSet.GetImageSize(ho_ImageWorld, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_ImageWorld, WindowControl.HalconWindow);
            HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 1);
            HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
            if (hv_Number == 1)
            {
                HOperatorSet.SmallestCircle(ho_SelectedRegions, out hv_Row3, out hv_Column3, out hv_Radius);
                HOperatorSet.GenCircle(out ho_Circle, hv_Row3, hv_Column3, hv_Radius);
                hv_x1 = (hv_Column3 * 0.000025) - 0.025;
                hv_y1 = (hv_Row3 * 0.000025) - 0.015;
                HOperatorSet.ReadPose(this.HandCameraPath + poseFile2, out hv_ObjInToolPose);
                HOperatorSet.PoseToHomMat3d(hv_ObjInToolPose, out hv_HomMat3D);
                HOperatorSet.AffineTransPoint3d(hv_HomMat3D, hv_x1, hv_y1, 0.003, out hv_Qx, out hv_Qy, out hv_Qz);
                double[] buf = new double[3];
                buf[0] = hv_Qx * 1000;
                buf[1] = hv_Qy * 1000;
                buf[2] = hv_Qz * 1000;
                markPosition = buf;
                errorCode = 0100;
                HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
                HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
                HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 1);
                HOperatorSet.DispObj(ho_Circle, WindowControl.HalconWindow);
                return true;


                //HOperatorSet.WriteImage(ho_ImageWorld, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_ImageWorld.bmp");
                //HOperatorSet.WriteImage(ho_Green, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_Green.bmp");
                //HOperatorSet.WriteImage(ho_ImageSmooth, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_ImageSmooth.bmp");
                //HOperatorSet.WriteImage(ho_ImageROI, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_ImageROI.bmp");

                //HOperatorSet.WriteRegion(ho_Region, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_Region.tif");
                //HOperatorSet.WriteRegion(ho_SelectedRegions, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_SelectedRegions.tif");
                //HOperatorSet.WriteRegion(ho_Circle, "D:/视觉系统/shiyanchengxu/gongxin/MARK位置检测/ho_Circle.tif");

            }
            else
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                markPosition = buf;
                errorCode = 0102;
                return false;
            }
        }
        /// <summary>
        /// 2. 电容料箱中电容中心位置计算与电容粗方向判定
        /// </summary>
        /// <param name="capacityPosition"></param>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0201下照相机无法获得图像
         *          0202用hough变换无法找到电容中心
         *          0203用hough变换得到的电容中心位置为（0,0）
         *          0204文件"电容粗方向检测模板编号.txt"不存在
         *          0205检测电容方向所需的模板文件不存在
         *          0206模板匹配不成功
         */
        public bool CapacityPosition(out double[] capacityPosition, out int errorCode)
        {
            HObject ho_Image;
            HObject ho_ImageWorld;
            HObject ho_Circle1, ho_Circle2, ho_Circle3, ho_Circle4, ho_Circle;
            HObject ho_ROI, ho_ROI2;
            HObject ho_ImageReduced, ho_ImageReduced2;
            HObject ho_ImageGauss;
            HObject ho_ImageEmphasize;
            HObject ho_ImaAmp, ho_ImaDir;
            HObject ho_GrayImage;
            HObject ho_Region, ho_ConnectedRegions, ho_SelectedRegions;
            HObject ho_RegionUnion;
            HObject ho_RegionClosing;
            HObject ho_Skeleton;
            HObject ho_Regionout;
            //HObject ho_GrayImage;
            //HObject ho_Red, ho_Green, ho_Blue;
            //HObject ho_Hue, ho_Saturation, ho_Intensity;
            //HObject ho_ImageNot;
            //HObject ho_ImageReduced;
            //HObject ho_ImageMean, ho_ImageSmooth;
            //HObject ho_ModelImage;
            //HObject ho_ModelImageWorld;
            //HObject ho_ModelImageMean;
            //HObject ho_ModelROI, ho_ModelImageROI;
            //HObject ho_ShapeModelImages, ho_ShapeModelRegions;
            HObject ho_ShapeModel;
            //HObject ho_ContCircle1;
            //HObject ho_Region, ho_ConnectedRegions, ho_SelectedRegions;
            //HObject ho_RegionClosing, ho_RegionOpening;
            //HObject ho_Contour;
            //HObject ho_ContCircle;
            HObject ho_ModelAtNewPosition;
            //HObject ho_ContCircle, ho_ContCircle1;
            //HObject ho_RegionOpening1, ho_RegionClosing1, ho_Contour1;
            //HObject ho_ImageNot1;
            //HObject ho_ImageMean1, ho_ImageSmooth1;
            //HObject ho_ImageReduced1;
            //HObject ho_Region1;
            //HObject ho_ConnectedRegions1, ho_SelectedRegions1;
            HTuple hv_Device = "";
            HTuple hv_AcqHandle;
            //HTuple hv_FileName = "D:/视觉系统/实际检测图片/qita/2016-07-17_19_53_23_351.bmp";
            //HTuple hv_CamParFile = "D:/视觉系统/shiyanchengxu/daheng/CameraParameters_down.dat";
            //HTuple hv_PoseFile1 = "D:/视觉系统/shiyanchengxu/daheng/CameraPoseNew_down.dat";
            //HTuple hv_PoseFile2 = "D:/视觉系统/shiyanchengxu/daheng/ObjInToolPose.dat";
            string camParFile = "CameraParameters_down.dat";
            string poseFile1 = "CameraPoseNew_down.dat";
            string poseFile2 = "Obj1InToolPose.dat";
            string shapeModel = "电容粗方向检测/";
            string modelTxt = "电容粗方向检测模板编号.txt";
            HTuple i;
            HTuple hv_CameraParameters;
            HTuple hv_CameraPoseNew1, hv_CameraPoseNew2;
            HTuple hv_Area;
            HTuple hv_Row = new HTuple(), hv_Column = new HTuple();
            //HTuple hv_Row1,hv_Column1,hv_Row2, hv_Column2;
            //HTuple hv_UsedThreshold;
            //HTuple hv_Number;
            //HTuple hv_Row, hv_Column, hv_Radius, hv_StartPhi, hv_EndPhi, hv_PointOrder;
            //HTuple hv_RowIntersection, hv_ColumnIntersection, hv_IsOverlapping;
            //HTuple hv_Angle1, hv_Angle2;
            HTuple hv_x1, hv_y1;
            //HTuple hv_AngleToX;
            HTuple hv_ObjInToolPose;
            HTuple hv_HomMat3D;
            HTuple hv_Qx, hv_Qy, hv_Qz;
            HTuple hv_HeightWin, hv_WidthWin;
            //HTuple hv_UsedThreshold1;
            //HTuple hv_AreaModelRegions, hv_RowModelRegions, hv_ColumnModelRegions;
            //HTuple hv_HeightPyramid;
            HTuple hv_FileHandle;
            HTuple hv_MN, hv_IsEOF;
            HTuple hv_ModelName = new HTuple();
            HTuple hv_ModelID;
            HTuple hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, hv_Score;
            HTuple hv_MovementOfObject;
            /*这些参数调试时可能要修改*/
            HTuple hv_Row0 = 830;
            HTuple hv_Column0 = 817;
            /*
             * 利用skeleton算法与hough变换计算电容中心位置
             */
            if (this.DeviceNumber == "1")
            {
                hv_Device = "2";
            }
            else if (this.DeviceNumber == "2")
            {
                hv_Device = "1";
            }
            try
            {
                //HOperatorSet.ReadImage(out ho_Image, hv_FileName);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", hv_Device, 1, -1, out hv_AcqHandle);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc1");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.01);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.42);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                double[] buf = new double[4];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                buf[3] = 0;
                capacityPosition = buf;
                errorCode = 0201;
                return false;
            }
            HOperatorSet.ReadCamPar(this.HandCameraPath + camParFile, out hv_CameraParameters);
            HOperatorSet.ReadPose(this.HandCameraPath + poseFile1, out hv_CameraPoseNew1);
            HOperatorSet.SetOriginPose(hv_CameraPoseNew1, -0.025, -0.016, 0, out hv_CameraPoseNew2);
            HOperatorSet.ImageToWorldPlane(ho_Image, out ho_ImageWorld, hv_CameraParameters, hv_CameraPoseNew2, 2000, 1500, 0.000025, "bilinear");
            //HOperatorSet.Rgb1ToGray(ho_ImageWorld, out ho_GrayImage);
            HOperatorSet.GetImageSize(ho_ImageWorld, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_ImageWorld, WindowControl.HalconWindow);
            HOperatorSet.GenCircle(out ho_Circle1, hv_Row0, hv_Column0, 700);//调试时可能要修改半径
            HOperatorSet.GenCircle(out ho_Circle2, hv_Row0, hv_Column0, 400);
            HOperatorSet.Difference(ho_Circle1, ho_Circle2, out ho_ROI);
            HOperatorSet.ReduceDomain(ho_ImageWorld, ho_ROI, out ho_ImageReduced);
            HOperatorSet.GaussFilter(ho_ImageReduced, out ho_ImageGauss, 7);
            HOperatorSet.Emphasize(ho_ImageGauss, out ho_ImageEmphasize, 7, 7, 2);
            HOperatorSet.EdgesColor(ho_ImageEmphasize, out ho_ImaAmp, out ho_ImaDir, "canny", 1, "nms", 20, 40);
            HOperatorSet.GenCircle(out ho_Circle3, hv_Row0, hv_Column0, 670);//调试时可能要修改半径
            HOperatorSet.GenCircle(out ho_Circle4, hv_Row0, hv_Column0, 420);
            HOperatorSet.Difference(ho_Circle3, ho_Circle4, out ho_ROI2);
            HOperatorSet.ReduceDomain(ho_ImaAmp, ho_ROI2, out ho_ImageReduced2);
            HOperatorSet.Rgb1ToGray(ho_ImageReduced2, out ho_GrayImage);
            HOperatorSet.Threshold(ho_GrayImage, out ho_Region, 30, 255);//调试时可能要修改灰度的阈值
            HOperatorSet.Connection(ho_Region, out ho_ConnectedRegions);
            HOperatorSet.SelectShape(ho_ConnectedRegions, out ho_SelectedRegions, "area", "and", 100, 99999999);//调试时可能要修改面积选择范围
            HOperatorSet.Union1(ho_SelectedRegions, out ho_RegionUnion);
            HOperatorSet.ClosingCircle(ho_RegionUnion, out ho_RegionClosing, 15);
            HOperatorSet.Skeleton(ho_RegionClosing, out ho_Skeleton);
            HOperatorSet.HoughCircles(ho_Skeleton, out ho_Regionout, 560, 7, 2);//调试时可能要修改半径
            HOperatorSet.AreaCenter(ho_Regionout, out hv_Area, out hv_Row, out hv_Column);
            try
            {
                HOperatorSet.GenCircle(out ho_Circle, hv_Row[0], hv_Column[0], 565);
            }
            catch
            {
                double[] buf = new double[4];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                buf[3] = 0;
                capacityPosition = buf;
                errorCode = 0202;
                return false;
            }
            HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 1);
            HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
            HOperatorSet.DispObj(ho_Circle, WindowControl.HalconWindow);
            if ((int)((new HTuple(((hv_Row.TupleSelect(0))).TupleEqual(0))).TupleAnd(new HTuple(((hv_Column.TupleSelect(0))).TupleEqual(0)))) != 0)
            {
                double[] buf = new double[4];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                buf[3] = 0;
                capacityPosition = buf;
                errorCode = 0203;
                return false;
            }
            //}
            //catch
            //{
            //    double[] buf = new double[3];
            //    buf[0] = 0;
            //    buf[1] = 0;
            //    buf[2] = 0;
            //    capacityPosition = buf;
            //    errorCode = 002;
            //    return false;
            //}

            //HOperatorSet.GenCircle(out ho_ROI, 820, 948, 700);
            //HOperatorSet.Decompose3(ho_ImageWorld, out ho_Red, out ho_Green, out ho_Blue);
            //HOperatorSet.TransFromRgb(ho_Red, ho_Green, ho_Blue, out ho_Hue, out ho_Saturation, out ho_Intensity, "hsv");
            //HOperatorSet.BitNot(ho_Saturation, out ho_ImageNot);
            //HOperatorSet.MeanImage(ho_ImageNot, out ho_ImageMean, 10, 10);
            //HOperatorSet.SmoothImage(ho_ImageMean, out ho_ImageSmooth, "deriche2", 20);
            ////hv_Row1 = 96;
            ////hv_Column1 = 66;
            ////hv_Row2 = 1448;
            ////hv_Column2 = 1290;
            ////HOperatorSet.Rectangle1Domain(ho_ImageSmooth, out ho_ImageReduced, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //HOperatorSet.BinaryThreshold(ho_ImageSmooth, out ho_Region, "max_separability", "dark", out hv_UsedThreshold);
            //HOperatorSet.Connection(ho_Region, out ho_ConnectedRegions);
            //HOperatorSet.SelectShape(ho_ConnectedRegions, out ho_SelectedRegions, "area", "and", 300000, 5000000);
            //HOperatorSet.CountObj(ho_SelectedRegions, out hv_Number);
            //HOperatorSet.GetImageSize(ho_ImageWorld, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            //HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            //HOperatorSet.DispObj(ho_ImageWorld, WindowControl.HalconWindow);
            //if (hv_Number == 1)
            //{
            //    HOperatorSet.ClosingCircle(ho_SelectedRegions, out ho_RegionClosing, 100);
            //    HOperatorSet.OpeningCircle(ho_RegionClosing, out ho_RegionOpening, 100);
            //    HOperatorSet.GenContourRegionXld(ho_RegionOpening, out ho_Contour, "center");
            //    HOperatorSet.FitCircleContourXld(ho_Contour, "atukey", -1, 0, 0, 3, 2, out hv_Row, out hv_Column, out hv_Radius, out hv_StartPhi, out hv_EndPhi, out hv_PointOrder);
            //    HOperatorSet.GenCircleContourXld(out ho_ContCircle, hv_Row, hv_Column, hv_Radius, hv_StartPhi, hv_EndPhi, hv_PointOrder, 1);
            //}
            //else
            //{
            //    double[] buf = new double[3];
            //    buf[0] = 0;
            //    buf[1] = 0;
            //    buf[2] = 0;
            //    capacityPosition = buf;
            //    errorCode = 002;
            //    return false;
            //}

            //if (hv_Number == 1)
            //{
            //HOperatorSet.ClosingCircle(ho_SelectedRegions, out ho_RegionClosing, 100);
            ////HOperatorSet.OpeningCircle(ho_RegionClosing, out ho_RegionOpening, 50);
            //HOperatorSet.GenContourRegionXld(ho_RegionClosing, out ho_Contour, "center");
            //HOperatorSet.FitCircleContourXld(ho_Contour, "atukey", -1, 0, 0, 3, 2, out hv_Row, out hv_Column, out hv_Radius, out hv_StartPhi, out hv_EndPhi, out hv_PointOrder);
            //HOperatorSet.GenCircleContourXld(out ho_ContCircle, hv_Row, hv_Column, hv_Radius, hv_StartPhi, hv_EndPhi, hv_PointOrder, 1);
            //获取电容粗方向
            //HOperatorSet.BitNot(ho_Intensity, out ho_ImageNot1);
            //HOperatorSet.MeanImage(ho_ImageNot1, out ho_ImageMean1, 5, 5);
            //HOperatorSet.SmoothImage(ho_ImageMean1, out ho_ImageSmooth1, "deriche2", 10);
            //HOperatorSet.Rectangle1Domain(ho_ImageSmooth1, out ho_ImageReduced1, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //HOperatorSet.BinaryThreshold(ho_ImageReduced1, out ho_Region1, "max_separability", "light", out hv_UsedThreshold1);
            //HOperatorSet.Connection(ho_Region1, out ho_ConnectedRegions1);
            //HOperatorSet.SelectShape(ho_ConnectedRegions1, out ho_SelectedRegions1, "area", "and", 100000, 1900000);
            //HOperatorSet.ClosingCircle(ho_SelectedRegions1, out ho_RegionClosing1, 60);
            //HOperatorSet.OpeningCircle(ho_RegionClosing1, out ho_RegionOpening1, 60);
            //HOperatorSet.GenContourRegionXld(ho_RegionOpening1, out ho_Contour1, "center");
            //HOperatorSet.OpeningCircle(ho_SelectedRegions, out ho_RegionOpening, 30);
            //HOperatorSet.ClosingCircle(ho_RegionOpening, out ho_RegionClosing1, 20);
            //HOperatorSet.GenContourRegionXld(ho_RegionClosing1, out ho_Contour1, "center");
            //HOperatorSet.GenCircleContourXld(out ho_ContCircle1, hv_Row, hv_Column, hv_Radius / 1.1, 0, 6.28318, "positive", 1);
            //HOperatorSet.IntersectionContoursXld(ho_Contour, ho_ContCircle1, "all", out hv_RowIntersection, out hv_ColumnIntersection, out hv_IsOverlapping);
            //if ((int)(new HTuple((new HTuple(hv_RowIntersection.TupleLength())).TupleEqual(0))) != 2)
            //{
            //HOperatorSet.AngleLx(hv_Row, hv_Column, hv_RowIntersection.TupleSelect(0), hv_ColumnIntersection.TupleSelect(0), out hv_Angle1);
            //HOperatorSet.AngleLx(hv_Row, hv_Column, hv_RowIntersection.TupleSelect(1), hv_ColumnIntersection.TupleSelect(1), out hv_Angle2);

            /*
             * 利用模板匹配判定电容粗方向
             */
            //try
            //{
            //    HOperatorSet.ReadImage(out ho_ModelImage, this.HandCameraPath + shapeModel);
            //}
            //catch
            //{
            //    double[] buf = new double[3];
            //    buf[0] = 0;
            //    buf[1] = 0;
            //    buf[2] = 0;
            //    capacityPosition = buf;
            //    errorCode = 003;
            //    return false;
            //}
            //HOperatorSet.ImageToWorldPlane(ho_ModelImage, out ho_ModelImageWorld, hv_CameraParameters, hv_CameraPoseNew2, 2000, 1500, 0.000025, "bilinear");
            //HOperatorSet.MeanImage(ho_ModelImageWorld, out ho_ModelImageMean, 10, 10);
            ////HOperatorSet.SmoothImage(ho_ModelImageMean, out ho_ModelImageSmooth, "deriche2", 20);
            //hv_Row1 = 470;
            //hv_Column1 = 1157;
            //hv_Row2 = 900;
            //hv_Column2 = 1402;
            //HOperatorSet.GenRectangle1(out ho_ModelROI, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //HOperatorSet.ReduceDomain(ho_ModelImageMean, ho_ModelROI, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 15);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, 8, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", "auto", 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);


            try
            {
                HOperatorSet.OpenFile(this.HandCameraPath + shapeModel + modelTxt, "input", out hv_FileHandle);
            }
            catch
            {
                double[] buf = new double[4];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                buf[3] = 0;
                capacityPosition = buf;
                errorCode = 0204;
                return false;
            }
            hv_IsEOF = 0;
            int j = -1;
            while (hv_IsEOF == 0)
            {
                HOperatorSet.FreadString(hv_FileHandle, out hv_MN, out hv_IsEOF);
                j = j + 1;
                if (hv_MN != "")
                {
                    hv_ModelName[j] = hv_MN;
                }
            }
            for (i = 0; i < j; i++)
            {
                try
                {
                    HOperatorSet.ReadShapeModel(this.HandCameraPath + shapeModel + hv_ModelName[i], out hv_ModelID);
                }
                catch
                {
                    double[] buf = new double[4];
                    buf[0] = 0;
                    buf[1] = 0;
                    buf[2] = 0;
                    buf[3] = 0;
                    capacityPosition = buf;
                    errorCode = 0205;
                    return false;
                }
                HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
                HOperatorSet.FindShapeModel(ho_ImageWorld, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.5, 1, 1, "least_squares", 0, 0.5, out hv_RowCheck, out hv_ColumnCheck, out hv_AngleCheck, out hv_Score);
                if ((int)(new HTuple((new HTuple(hv_RowCheck.TupleLength())).TupleEqual(1))) != 0)
                {
                    HOperatorSet.VectorAngleToRigid(0, 0, 0, hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, out hv_MovementOfObject);
                    HOperatorSet.AffineTransContourXld(ho_ShapeModel, out ho_ModelAtNewPosition, hv_MovementOfObject);
                    HOperatorSet.DispObj(ho_ModelAtNewPosition, WindowControl.HalconWindow);
                    hv_x1 = (hv_Column.TupleSelect(0) * 0.000025) - 0.025;
                    hv_y1 = (hv_Row.TupleSelect(0) * 0.000025) - 0.016;
                    HOperatorSet.ReadPose(this.HandCameraPath + poseFile2, out hv_ObjInToolPose);
                    HOperatorSet.PoseToHomMat3d(hv_ObjInToolPose, out hv_HomMat3D);
                    HOperatorSet.AffineTransPoint3d(hv_HomMat3D, hv_x1, hv_y1, 0.003, out hv_Qx, out hv_Qy, out hv_Qz);
                    double[] buf = new double[4];
                    buf[0] = hv_Qx * 1000;
                    buf[1] = hv_Qy * 1000;
                    buf[2] = (hv_AngleCheck * 180) / 3.1415926;
                    buf[3] = hv_Qz * 1000;
                    capacityPosition = buf;
                    errorCode = 0200;
                    return true;
                }
            }
            double[] tuf = new double[4];
            tuf[0] = 0;
            tuf[1] = 0;
            tuf[2] = 0;
            tuf[3] = 0;
            capacityPosition = tuf;
            errorCode = 0206;
            return false;

            //else
            //{
            //    HOperatorSet.VectorAngleToRigid(0, 0, 0, hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, out hv_MovementOfObject);
            //    HOperatorSet.AffineTransContourXld(ho_ShapeModel, out ho_ModelAtNewPosition, hv_MovementOfObject);
            //    hv_x1 = (hv_Column.TupleSelect(0) * 0.000025) - 0.025;
            //    hv_y1 = (hv_Row.TupleSelect(0) * 0.000025) - 0.016;
            //    HOperatorSet.ReadPose(this.HandCameraPath + poseFile2, out hv_ObjInToolPose);
            //    HOperatorSet.PoseToHomMat3d(hv_ObjInToolPose, out hv_HomMat3D);
            //    HOperatorSet.AffineTransPoint3d(hv_HomMat3D, hv_x1, hv_y1, 0.003, out hv_Qx, out hv_Qy, out hv_Qz);
            //    double[] tuf = new double[3];
            //    tuf[0] = hv_Qx;
            //    tuf[1] = hv_Qy;
            //    tuf[2] = (hv_AngleCheck * 180) / 3.1415926;
            //    capacityPosition = tuf;
            //    errorCode = 000;
            //    //HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //    //HOperatorSet.DispObj(ho_ContCircle, WindowControl.HalconWindow);
            //    HOperatorSet.DispObj(ho_ModelAtNewPosition, WindowControl.HalconWindow);
            //    return true;
            //}

            //hv_x1 = (hv_Column * 0.000025) - 0.025;
            //        hv_y1 = (hv_Row * 0.000025) - 0.015;
            //        if (hv_Angle1 >= 1.5708 && hv_Angle2 <= -1.5708)
            //        {
            //            hv_AngleToX = (hv_Angle1 + hv_Angle2 + 6.2832) / 2;
            //        }
            //        else if (hv_Angle1 <= -1.5708 && hv_Angle2 >= 1.5708)
            //        {
            //            hv_AngleToX = (hv_Angle1 + hv_Angle2 + 6.2832) / 2;
            //        }
            //        else
            //        {
            //            hv_AngleToX = (hv_Angle1 + hv_Angle2) / 2;
            //        }


            //HOperatorSet.DispObj(ho_ContCircle, WindowControl.HalconWindow);


            //HOperatorSet.WriteImage(ho_ImageWorld, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/01ImageWorld.bmp");
            //HOperatorSet.WriteImage(ho_Red, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/02Red.bmp");
            //HOperatorSet.WriteImage(ho_Saturation, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/03Saturation.bmp");
            //HOperatorSet.WriteImage(ho_ImageNot, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/04ImageNot.bmp");
            //HOperatorSet.WriteImage(ho_ImageSmooth, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/05ImageSmooth.bmp");
            //HOperatorSet.WriteImage(ho_ImageReduced, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/06ImageReduced.bmp");
            //HOperatorSet.WriteImage(ho_Intensity, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/10ho_Intensity.bmp");
            //HOperatorSet.WriteImage(ho_ImageNot1, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/11ho_ImageNot1.bmp");
            //HOperatorSet.WriteImage(ho_ImageSmooth1, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/12ho_ImageSmooth1.bmp");
            //HOperatorSet.WriteImage(ho_ImageReduced1, "bmp", 0, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/13ho_ImageReduced1.bmp");


            //HOperatorSet.WriteRegion(ho_Region, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/07ho_Region.tif");
            //HOperatorSet.WriteRegion(ho_SelectedRegions, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/08ho_SelectedRegions.tif");
            //HOperatorSet.WriteRegion(ho_RegionClosing, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/09ho_RegionClosing.tif");
            //HOperatorSet.WriteRegion(ho_Region1, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/14ho_Region1.tif");
            //HOperatorSet.WriteRegion(ho_SelectedRegions1, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/15ho_SelectedRegions1.tif");
            //HOperatorSet.WriteRegion(ho_RegionClosing1, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/16ho_RegionClosing1.tif");
            //HOperatorSet.WriteRegion(ho_RegionOpening1, "D:/视觉系统/shiyanchengxu/gongxin/电容圆心与方向检测/17ho_RegionOpening1.tif");


            //}
            //else
            //{
            //    double[] buf = new double[3];
            //    buf[0] = 0;
            //    buf[1] = 0;
            //    buf[2] = 0;
            //    capacityPosition = buf;
            //    errorCode = 003;
            //    return false;
            //}
            //}
            //else
            //{
            //    double[] buf = new double[3];
            //    buf[0] = 0;
            //    buf[1] = 0;
            //    buf[2] = 0;
            //    capacityPosition = buf;
            //    errorCode = 002;
            //    return false;
            //}


        }
        /// <summary>
        /// 3. PCB板元件极性检查
        /// </summary>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0301下照相机无法获得要检测的图片
         *          0302文件"PCB板元件极性检查模板编号.txt"不存在
         *          0303模板文件不存在
         *          0304电容极性错误
         *          0305电容存在缺失
         *          
         *          
         */
        public bool IsCapacityRight(out int errorCode)
        {

            //HObject ho_ModelImage;
            //HObject ho_ModelImageMean;
            //HObject ho_ModelROI, ho_ModelImageROI;
            //HObject ho_ShapeModelImages, ho_ShapeModelRegions;
            HObject ho_ShapeModel;
            HObject ho_Image;
            HObject ho_ImageMean;
            HObject ho_ModelAtNewPosition;
            HObject ho_Rectangle;

            HTuple hv_Device = "";
            HTuple hv_AcqHandle;
            //HTuple hv_FileName1 = "D:/视觉系统/实际检测图片/qita/2016-07-13_13_38_10_391.bmp";
            //HTuple hv_FileName2 = "D:/视觉系统/实际检测图片/qita/2016-07-13_13_41_07_967.bmp";
            //HTuple hv_CamParFile = "D:/视觉系统/shiyanchengxu/daheng/CameraParameters_down.dat";
            //HTuple hv_PoseFile1 = "D:/视觉系统/shiyanchengxu/daheng/CameraPoseNew_down.dat";
            //HTuple hv_PoseFile2 = "D:/视觉系统/shiyanchengxu/daheng/ObjInToolPose.dat";
            string shapeModel = "PCB板元件极性检查/";
            string modelTxt = "PCB板元件极性检查模板编号.txt";
            HTuple i;
            HTuple hv_Row1 = 1130, hv_Column1 = 311, hv_Row2 = 1885, hv_Column2 = 1069;
            //HTuple hv_AreaModelRegions, hv_RowModelRegions, hv_ColumnModelRegions;
            //HTuple hv_HeightPyramid;
            HTuple hv_FileHandle;
            HTuple hv_MN, hv_IsEOF;
            HTuple hv_ModelName = new HTuple();
            HTuple hv_ModelID;
            HTuple hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, hv_Score;
            HTuple hv_MovementOfObject;
            HTuple hv_HeightWin, hv_WidthWin;
            HTuple hv_RowCapacity = 1360, hv_ColumnCapacity = 1220;


            //try 
            //{
            //    HOperatorSet.ReadImage(out ho_ModelImage, this.HandCameraPath + shapeModel);
            //}
            //catch
            //{
            //    errorCode = 001;
            //    return false;
            //}
            //HOperatorSet.MeanImage(ho_ModelImage, out ho_ModelImageMean, 10, 10);
            //HOperatorSet.SmoothImage(ho_ModelImageMean, out ho_ModelImageSmooth, "deriche2", 10);
            //hv_Row1 = 1130;
            //hv_Column1 = 311;
            //hv_Row2 = 1885;
            //hv_Column2 = 1069;
            //HOperatorSet.GenRectangle1(out ho_ModelROI, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //HOperatorSet.ReduceDomain(ho_ModelImageMean, ho_ModelROI, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, 8, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
            if (this.DeviceNumber == "1")
            {
                hv_Device = "2";
            }
            else if (this.DeviceNumber == "2")
            {
                hv_Device = "1";
            }
            try
            {
                //HOperatorSet.ReadImage(out ho_Image1, hv_FileName2);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", hv_Device, 1, -1, out hv_AcqHandle);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc2");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.31);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.31);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                errorCode = 0301;
                return false;
            }
            HOperatorSet.MeanImage(ho_Image, out ho_ImageMean, 10, 10);
            HOperatorSet.GetImageSize(ho_Image, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_Image, WindowControl.HalconWindow);
            //HOperatorSet.SmoothImage(ho_ImageMean1, out ho_ImageSmooth1, "deriche2", 10);

            try
            {
                HOperatorSet.OpenFile(this.HandCameraPath + shapeModel + modelTxt, "input", out hv_FileHandle);
            }
            catch
            {
                errorCode = 0302;
                return false;
            }
            hv_IsEOF = 0;
            int j = -1;
            while (hv_IsEOF == 0)
            {
                HOperatorSet.FreadString(hv_FileHandle, out hv_MN, out hv_IsEOF);
                j = j + 1;
                if (hv_MN != "")
                {
                    hv_ModelName[j] = hv_MN;
                }
            }

            for (i = 0; i < j; i++)
            {
                try
                {
                    HOperatorSet.ReadShapeModel(this.HandCameraPath + shapeModel + hv_ModelName[i], out hv_ModelID);
                }
                catch
                {
                    errorCode = 0303;
                    return false;
                }
                HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
                HOperatorSet.FindShapeModel(ho_ImageMean, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.1, 1, 0.7, "least_squares", 0, 0.7, out hv_RowCheck, out hv_ColumnCheck, out hv_AngleCheck, out hv_Score);
                if ((int)(new HTuple((new HTuple(hv_RowCheck.TupleLength())).TupleEqual(1))) != 0)
                {
                    HOperatorSet.VectorAngleToRigid(0, 0, 0, hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, out hv_MovementOfObject);
                    HOperatorSet.AffineTransContourXld(ho_ShapeModel, out ho_ModelAtNewPosition, hv_MovementOfObject);
                    HOperatorSet.DispObj(ho_ModelAtNewPosition, WindowControl.HalconWindow);
                    if (hv_AngleCheck > 5.4978 || hv_AngleCheck < 0.7853)
                    {
                        errorCode = 0300;
                        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
                        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
                        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
                        HOperatorSet.GenRectangle1(out ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
                        HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
                        return true;
                    }
                    else
                    {
                        errorCode = 0304;
                        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
                        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
                        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
                        HOperatorSet.GenRectangle1(out ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
                        HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
                        return false;
                    }
                }
            }
            errorCode = 0305;
            HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
            HOperatorSet.GenRectangle1(out ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
            return false;


            //HOperatorSet.FindShapeModel(ho_ImageMean1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.1, 1, 0.7, "least_squares", 0, 0.7, out hv_RowCheck, out hv_ColumnCheck, out hv_AngleCheck, out hv_Score);
            //HOperatorSet.GetImageSize(ho_Image1, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            //HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            //HOperatorSet.DispObj(ho_Image1, WindowControl.HalconWindow);
            //if ((int)(new HTuple((new HTuple(hv_RowCheck.TupleLength())).TupleEqual(0))) != 0)
            //{
            //    errorCode=003;
            //    HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //    HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //    HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
            //    //HOperatorSet.GenRectangle2(out ho_Rectangle, hv_RowCapacity, hv_ColumnCapacity, 0, 200, 200);
            //    HOperatorSet.GenRectangle1(out ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //    HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
            //    return false;
            //}
            //else
            //{
            //    HOperatorSet.VectorAngleToRigid(0, 0, 0, hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, out hv_MovementOfObject);
            //    HOperatorSet.AffineTransContourXld(ho_ShapeModel, out ho_ModelAtNewPosition, hv_MovementOfObject);
            //    if (hv_AngleCheck > 5.4978 || hv_AngleCheck < 0.7853)
            //    {
            //        errorCode = 000;
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
            //        //HOperatorSet.GenRectangle2(out ho_Rectangle, hv_RowCapacity, hv_ColumnCapacity, 0, 200, 200);
            //        HOperatorSet.GenRectangle1(out ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //        HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
            //        return true;
            //    }
            //    else
            //    {
            //        errorCode = 004;
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
            //        //HOperatorSet.GenRectangle2(out ho_Rectangle, hv_RowCapacity, hv_ColumnCapacity, 0, 200, 200);
            //        HOperatorSet.GenRectangle1(out ho_Rectangle, hv_Row1, hv_Column1, hv_Row2, hv_Column2);
            //        HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
            //        return false;
            //    }
            //}
            ////else
            //{
            //    errorCode = 004;
            //    HOperatorSet.GetImageSize(ho_Image1, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            //    HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            //    HOperatorSet.DispObj(ho_Image1, WindowControl.HalconWindow);
            //    HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //    HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //    HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 2);
            //    HOperatorSet.GenRectangle2(out ho_Rectangle, 1360, 1220, 0, 200, 200);
            //    HOperatorSet.DispObj(ho_Rectangle, WindowControl.HalconWindow);
            //    return false;
            //}

        }
        /// <summary>
        /// 4. PCB元件缺失检查
        /// </summary>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0401下照相机无法获得要检测的图片
         *          0402元件检测匹配区域ROI.txt文件缺失
         *          0403包含PCB元件缺失检查模板名的txt文件缺失
         *          0404模板文件不存在
         *          0405元件存在缺失
         */
        public bool IsComponentMissing(out int errorCode)
        {
            //HObject ho_ModelImage;
            //HObject ho_ModelImageMean;
            //HObject ho_ModelROI1;
            //HObject ho_ModelImageROI;
            //HObject ho_ShapeModelImages, ho_ShapeModelRegions;
            HObject ho_ShapeModel;
            HObject ho_Image;
            HObject ho_ImageMean;
            HObject ho_ROI, ho_ImageROI;

            HTuple hv_Device = "";
            HTuple hv_AcqHandle;
            //HTuple hv_FileName1 = "D:/视觉系统/实际检测图片/qita/2016-07-13_13_38_10_391.bmp";
            //HTuple hv_FileName2 = "D:/视觉系统/实际检测图片/qita/2016-07-13_13_38_10_391 - 副本.bmp";
            //HTuple hv_CamParFile = "D:/视觉系统/shiyanchengxu/daheng/CameraParameters_down.dat";
            //HTuple hv_PoseFile1 = "D:/视觉系统/shiyanchengxu/daheng/CameraPoseNew_down.dat";
            //HTuple hv_PoseFile2 = "D:/视觉系统/shiyanchengxu/daheng/ObjInToolPose.dat";
            string shapeModel = "PCB元件缺失检查/";
            string modelTxt;
            //string modelROI = "元件检测模板ROI.txt";
            string regionROI = "元件检测匹配区域ROI.txt";
            HTuple hv_FileHandle;
            HTuple i, j;
            HTuple hv_Number, hv_Index, hv_IsEOF, hv_MN;
            HTuple hv_ModelName = new HTuple();
            HTuple hv_R1, hv_C1, hv_R2, hv_C2, hv_CN;
            HTuple hv_Row1 = new HTuple(), hv_Column1 = new HTuple(), hv_Row2 = new HTuple(), hv_Column2 = new HTuple(), hv_ClassNumber = new HTuple();
            //HTuple hv_R3, hv_C3, hv_R4, hv_C4;
            //HTuple hv_Row3 = new HTuple(), hv_Column3 = new HTuple(), hv_Row4 = new HTuple(), hv_Column4 = new HTuple();


            //HTuple hv_AreaModelRegions, hv_RowModelRegions, hv_ColumnModelRegions;
            //HTuple hv_HeightPyramid;
            HTuple hv_ModelID;
            HTuple hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, hv_Score;
            //HTuple hv_RowCheck2, hv_ColumnCheck2, hv_AngleCheck2, hv_Score2;
            //HTuple hv_RowCheck3, hv_ColumnCheck3, hv_AngleCheck3, hv_Score3;
            //HTuple hv_RowCheck4, hv_ColumnCheck4, hv_AngleCheck4, hv_Score4;
            //HTuple hv_RowCheck5, hv_ColumnCheck5, hv_AngleCheck5, hv_Score5;
            //HTuple hv_RowCheck6, hv_ColumnCheck6, hv_AngleCheck6, hv_Score6;
            HTuple hv_HeightWin, hv_WidthWin;

            //try
            //{
            //    HOperatorSet.ReadImage(out ho_ModelImage, this.HandCameraPath + shapeModel);
            //}
            //catch
            //{
            //    errorCode = 001;
            //    return false;
            //}
            //HOperatorSet.MeanImage(ho_ModelImage, out ho_ModelImageMean, 10, 10);
            //HOperatorSet.SmoothImage(ho_ModelImageMean, out ho_ModelImageSmooth, "deriche2", 10);
            /*  
             * 建立第一个模板*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI1, 826, 891, 1095, 1594);//第一个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI1, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID1);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel1, hv_ModelID1, 1);
            ///*  
            // * 建立第二个模板*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI2, 1043, 518, 1571, 732);//第二个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI2, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID2);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel2, hv_ModelID2, 1);
            ///*  
            // * 建立第三个模板*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI3, 1662, 923, 1892, 1419);//第三个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI3, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID3);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel3, hv_ModelID3, 1);
            if (this.DeviceNumber == "1")
            {
                hv_Device = "2";
            }
            else if (this.DeviceNumber == "2")
            {
                hv_Device = "1";
            }
            try
            {
                //HOperatorSet.ReadImage(out ho_Image1, hv_FileName2);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", hv_Device, 1, -1, out hv_AcqHandle);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc2");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.36);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.38);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                errorCode = 0401;
                return false;
            }
            HOperatorSet.GetImageSize(ho_Image, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_Image, WindowControl.HalconWindow);
            HOperatorSet.MeanImage(ho_Image, out ho_ImageMean, 10, 10);
            //try
            //{
            //    HOperatorSet.OpenFile(this.HandCameraPath + modelROI, "input", out hv_FileHandle);
            //}
            //catch
            //{
            //    errorCode = 003;
            //    return false;
            //}
            //HOperatorSet.FreadString(hv_FileHandle, out hv_Number, out hv_IsEOF);
            ////hv_Number = hv_Number.TupleNumber();
            //hv_IsEOF = 0;
            //int i;
            //while (hv_IsEOF == 0)
            //{
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_Index, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_R1, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_C1, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_R2, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_C2, out hv_IsEOF);
            //    hv_Index = hv_Index.TupleNumber();
            //    try
            //    {
            //        i = hv_Index - 1;
            //        hv_Row1[i] = hv_R1.TupleNumber();
            //        hv_Column1[i] = hv_C1.TupleNumber();
            //        hv_Row2[i] = hv_R2.TupleNumber();
            //        hv_Column2[i] = hv_C2.TupleNumber();
            //    }
            //    catch { }
            //}
            /*
             * 读取元件检测匹配区域ROI.txt
             */
            try
            {
                HOperatorSet.OpenFile(this.HandCameraPath + shapeModel + regionROI, "input", out hv_FileHandle);
            }
            catch
            {
                errorCode = 0402;
                return false;
            }
            HOperatorSet.FreadString(hv_FileHandle, out hv_Number, out hv_IsEOF);
            hv_IsEOF = 0;
            int index = 0;
            while (hv_IsEOF == 0)
            {
                HOperatorSet.FreadString(hv_FileHandle, out hv_Index, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_R1, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_C1, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_R2, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_C2, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_CN, out hv_IsEOF);
                hv_Index = hv_Index.TupleNumber();
                try
                {
                    index = hv_Index - 1;
                    hv_Row1[index] = hv_R1.TupleNumber();
                    hv_Column1[index] = hv_C1.TupleNumber();
                    hv_Row2[index] = hv_R2.TupleNumber();
                    hv_Column2[index] = hv_C2.TupleNumber();
                    hv_ClassNumber[index] = hv_CN.TupleNumber();
                }
                catch { }
            }
            int k = 0;
            for (i = 0; i <= index; i++)
            {
                //HOperatorSet.GenRectangle1(out ho_ROI, hv_Row1[i], hv_Column1[i], hv_Row2[i], hv_Column2[i]);
                //HOperatorSet.ReduceDomain(ho_ModelImage, ho_ModelROI1, out ho_ModelImageROI);
                //HOperatorSet.CreateShapeModel(ho_ModelImageROI, 8, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
                //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
                HOperatorSet.GenRectangle1(out ho_ROI, hv_Row1[i], hv_Column1[i], hv_Row2[i], hv_Column2[i]);
                HOperatorSet.ReduceDomain(ho_ImageMean, ho_ROI, out ho_ImageROI);
                int m = hv_ClassNumber[i];
                switch (m)
                {
                    case 1:
                        {
                            modelTxt = "PCB元件缺失检查01.txt";
                            break;
                        }
                    case 2:
                        {
                            modelTxt = "PCB元件缺失检查02.txt";
                            break;
                        }
                    case 3:
                        {
                            modelTxt = "PCB元件缺失检查03.txt";
                            break;
                        }
                    default:
                        {
                            modelTxt = "";
                            break;
                        }
                }

                try
                {
                    HOperatorSet.OpenFile(this.HandCameraPath + shapeModel + modelTxt, "input", out hv_FileHandle);
                }
                catch
                {
                    errorCode = 0403;
                    return false;
                }
                hv_IsEOF = 0;
                int n = -1;
                while (hv_IsEOF == 0)
                {
                    HOperatorSet.FreadString(hv_FileHandle, out hv_MN, out hv_IsEOF);
                    n = n + 1;
                    if (hv_MN != "")
                    {
                        hv_ModelName[n] = hv_MN;
                    }
                }

                for (j = 0; j < n; j = j + 1)
                {
                    try
                    {
                        HOperatorSet.ReadShapeModel(this.HandCameraPath + shapeModel + hv_ModelName[j], out hv_ModelID);
                    }
                    catch
                    {
                        errorCode = 0404;
                        return false;
                    }
                    HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
                    HOperatorSet.FindShapeModel(ho_ImageROI, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.2, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck, out hv_ColumnCheck, out hv_AngleCheck, out hv_Score);
                    if ((int)(new HTuple((new HTuple(hv_RowCheck.TupleLength())).TupleEqual(1))) != 0)
                    {
                        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
                        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
                        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
                        HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
                        break;
                    }
                    else if (j == n - 1)
                    {
                        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
                        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
                        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
                        HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
                        k = k + 1;
                    }
                }
            }
            if (k == 0)
            {
                errorCode = 0400;
                return true;
            }
            else
            {
                errorCode = 0405;
                return false;
            }

            //    if ((int)(new HTuple((new HTuple(hv_RowCheck1.TupleLength())).TupleEqual(0))) != 0)
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI1, WindowControl.HalconWindow);
            //        k = k + 1;
            //    }
            //    else
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI1, WindowControl.HalconWindow);
            //    }
            //}
            //if (k == 0)
            //{
            //    errorCode = 000;
            //    return true;
            //}
            //else
            //{
            //    errorCode = 005;
            //    return false;
            //}


            ///*  
            // * 对第一个元件进行模板匹配*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI1, 826, 891, 1095, 1594);//第一个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI1, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);


            //HOperatorSet.SmoothImage(ho_ImageMean1, out ho_ImageSmooth1, "deriche2", 10);
            //HOperatorSet.GenRectangle1(out ho_ROI1, 735, 780, 1195, 1775);
            //HOperatorSet.ReduceDomain(ho_ImageSmooth1, ho_ROI1, out ho_ImageROI1);
            //HOperatorSet.FindShapeModel(ho_ImageROI1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.2, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck1, out hv_ColumnCheck1, out hv_AngleCheck1, out hv_Score1);
            ///*  
            // * 对第二个元件进行模板匹配*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI2, 1088, 891, 1357, 1603);//第二个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI2, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
            //HOperatorSet.GenRectangle1(out ho_ROI1, 1023, 803, 1435, 1746);
            //HOperatorSet.ReduceDomain(ho_ImageSmooth1, ho_ROI1, out ho_ImageROI1);
            //HOperatorSet.FindShapeModel(ho_ImageROI1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.2, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck2, out hv_ColumnCheck2, out hv_AngleCheck2, out hv_Score2);
            ///*  
            // * 对第三个元件进行模板匹配*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI3, 1383, 712, 1652, 1438);//第三个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI3, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
            //HOperatorSet.GenRectangle1(out ho_ROI1, 1312, 628, 1717, 1532);
            //HOperatorSet.ReduceDomain(ho_ImageSmooth1, ho_ROI1, out ho_ImageROI1);
            //HOperatorSet.FindShapeModel(ho_ImageROI1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.2, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck3, out hv_ColumnCheck3, out hv_AngleCheck3, out hv_Score3);
            ///*  
            // * 对第四个元件进行模板匹配*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI4, 1043, 518, 1571, 732);//第四个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI4, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
            //HOperatorSet.GenRectangle1(out ho_ROI1, 819, 359, 1713, 810);
            //HOperatorSet.ReduceDomain(ho_ImageSmooth1, ho_ROI1, out ho_ImageROI1);
            //HOperatorSet.FindShapeModel(ho_ImageROI1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.1, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck4, out hv_ColumnCheck4, out hv_AngleCheck4, out hv_Score4);
            ///*  
            // * 对第五个元件进行模板匹配*/
            //HOperatorSet.GenRectangle1(out ho_ModelROI5, 1662, 923, 1892, 1419);//第五个模板的区域
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI5, out ho_ModelImageROI);
            //HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            //HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            //HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            //HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            //HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
            //HOperatorSet.GenRectangle1(out ho_ROI1, 1581, 826, 1918, 1500);
            //HOperatorSet.ReduceDomain(ho_ImageSmooth1, ho_ROI1, out ho_ImageROI1);
            //HOperatorSet.FindShapeModel(ho_ImageROI1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.05, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck5, out hv_ColumnCheck5, out hv_AngleCheck5, out hv_Score5);
            ///*  
            // * 对第六个元件进行模板匹配*/
            ////HOperatorSet.GenRectangle1(out ho_ModelROI6, 1370, 1558, 1613, 1811);//第六个模板的区域
            ////HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI6, out ho_ModelImageROI);
            ////HOperatorSet.InspectShapeModel(ho_ModelImageROI, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
            ////HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
            ////HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
            ////HOperatorSet.CreateShapeModel(ho_ModelImageROI, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
            ////HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
            ////HOperatorSet.GenRectangle1(out ho_ROI1, 1159, 1354, 1730, 1895);
            ////HOperatorSet.ReduceDomain(ho_ImageSmooth1, ho_ROI1, out ho_ImageROI1);
            ////HOperatorSet.FindShapeModel(ho_ImageROI1, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.6, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck6, out hv_ColumnCheck6, out hv_AngleCheck6, out hv_Score6);
            //HOperatorSet.GetImageSize(ho_Image1, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            //HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            //HOperatorSet.DispObj(ho_Image1, WindowControl.HalconWindow);
            ////HOperatorSet.GenRectangle1(out ho_ModelROI1a, 1088, 891, 1357, 1603);
            ////HOperatorSet.GenRectangle1(out ho_ModelROI1b, 1383, 712, 1652, 1438);
            //if ((int)(new HTuple((new HTuple(hv_RowCheck1.TupleLength())).TupleEqual(1))) != 0 && 
            //    (int)(new HTuple((new HTuple(hv_RowCheck2.TupleLength())).TupleEqual(1))) != 0 &&
            //    (int)(new HTuple((new HTuple(hv_RowCheck3.TupleLength())).TupleEqual(1))) != 0 && 
            //    (int)(new HTuple((new HTuple(hv_RowCheck4.TupleLength())).TupleEqual(1))) != 0 &&
            //    (int)(new HTuple((new HTuple(hv_RowCheck5.TupleLength())).TupleEqual(1))) != 0 )
            //{
            //    errorCode = 000;
            //    HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //    HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //    HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //    HOperatorSet.DispObj(ho_ModelROI1, WindowControl.HalconWindow);
            //    HOperatorSet.DispObj(ho_ModelROI2, WindowControl.HalconWindow);
            //    HOperatorSet.DispObj(ho_ModelROI3, WindowControl.HalconWindow);
            //    HOperatorSet.DispObj(ho_ModelROI4, WindowControl.HalconWindow);
            //    HOperatorSet.DispObj(ho_ModelROI5, WindowControl.HalconWindow);
            //    return true;
            //}
            //else
            //{
            //    if ((int)(new HTuple((new HTuple(hv_RowCheck1.TupleLength())).TupleEqual(0))) != 0)
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI1, WindowControl.HalconWindow);
            //    }
            //    else
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI1, WindowControl.HalconWindow);
            //    }

            //    if ((int)(new HTuple((new HTuple(hv_RowCheck2.TupleLength())).TupleEqual(0))) != 0)
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI2, WindowControl.HalconWindow);
            //    }
            //    else
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI2, WindowControl.HalconWindow);
            //    }

            //    if ((int)(new HTuple((new HTuple(hv_RowCheck3.TupleLength())).TupleEqual(0))) != 0)
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI3, WindowControl.HalconWindow);
            //    }
            //    else
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI3, WindowControl.HalconWindow);
            //    }

            //    if ((int)(new HTuple((new HTuple(hv_RowCheck4.TupleLength())).TupleEqual(0))) != 0)
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI4, WindowControl.HalconWindow);
            //    }
            //    else
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI4, WindowControl.HalconWindow);
            //    }

            //    if ((int)(new HTuple((new HTuple(hv_RowCheck5.TupleLength())).TupleEqual(0))) != 0)
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI5, WindowControl.HalconWindow);
            //    }
            //    else
            //    {
            //        HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //        HOperatorSet.DispObj(ho_ModelROI5, WindowControl.HalconWindow);
            //    }

            //if ((int)(new HTuple((new HTuple(hv_RowCheck6.TupleLength())).TupleEqual(0))) != 0)
            //{
            //    HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //    HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //    HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //    HOperatorSet.DispObj(ho_ModelROI6, WindowControl.HalconWindow);
            //}
            //else
            //{
            //    HOperatorSet.SetColor(WindowControl.HalconWindow, "green");
            //    HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //    HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //    HOperatorSet.DispObj(ho_ModelROI6, WindowControl.HalconWindow);
            //}
            //    errorCode = 003;
            //    return false;
            //}
        }
        /// <summary>
        /// 5. 电容引脚质量检测与精方向判定
        /// </summary>
        /// <param name="rawAngleToX"></param>
        /// <param name="capacityPinPosition"></param>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0501上照相机无法获得图像
         *          0502检测电容引脚时得到连通区域的个数不为2
         *          0503引脚投影的外接圆半径过大
         *          0504引脚间距不合格
         *          0505精方向与粗方向之间近似垂直，导致引脚极性无法确定
         */
        public bool CapacityPinPosition(double rawAngleToX, out double[] capacityPinPosition, out int errorCode)
        {
            HObject ho_Image;
            HObject ho_ImageWorld;
            //HObject ho_GrayImage;
            HObject ho_ImageMean;
            HObject ho_ROI, ho_ImageROI;
            HObject ho_Region;
            HObject ho_RegionClosing;
            HObject ho_ConnectedRegions, ho_SelectedRegions;
            HObject ho_Circle;
            HObject ho_Red, ho_Green, ho_Blue;
            HObject ho_Hue, ho_Saturation, ho_Intensity;
            //HObject ho_ImageNot;

            HTuple hv_AcqHandle;
            //HTuple hv_FileName = "D:/视觉系统/实际检测图片/qita/2016-07-12_16_53_36_304.bmp";
            //HTuple hv_CamParFile = "D:/视觉系统/shiyanchengxu/daheng/CameraParameters_up.dat";
            //HTuple hv_PoseFile1 = "D:/视觉系统/shiyanchengxu/daheng/CameraPoseNew_up.dat";
            //HTuple hv_PoseFile2 = "D:/视觉系统/shiyanchengxu/daheng/ObjInToolPose.dat";
            string camParFile = "CameraParameters_up.dat";
            string poseFile1 = "CameraPoseNew_up.dat";
            string poseFile2 = "Obj2InToolPose.dat";
            HTuple hv_CameraParameters;
            HTuple hv_CameraPoseNew1, hv_CameraPoseNew2;
            HTuple hv_Row, hv_Column, hv_RadiusROI;
            HTuple hv_UsedThreshold;
            HTuple hv_Row3, hv_Column3, hv_Radius;
            HTuple hv_Distance;
            HTuple hv_Angle;
            HTuple hv_AngleToX, hv_AngleToX0;
            HTuple hv_preciseAngleToX;
            HTuple hv_x1, hv_y1;
            HTuple hv_ObjInToolPose;
            HTuple hv_HomMat3D;
            HTuple hv_Qx, hv_Qy, hv_Qz;
            HTuple hv_HeightWin, hv_WidthWin;

            try
            {
                //HOperatorSet.ReadImage(out ho_Image, hv_FileName);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", this.DeviceNumber, 1, -1, out hv_AcqHandle);//同时接两个相机的时候，下照相机的Device为"1"，上照相机的Device为"2"
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc2");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2000);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter_unit", "us");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.18254);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.58);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                capacityPinPosition = buf;
                errorCode = 0501;
                return false;
            }
            HOperatorSet.ReadCamPar(this.EyeCameraPath + camParFile, out hv_CameraParameters);
            HOperatorSet.ReadPose(this.EyeCameraPath + poseFile1, out hv_CameraPoseNew1);
            HOperatorSet.SetOriginPose(hv_CameraPoseNew1, -0.025, -0.015, 0, out hv_CameraPoseNew2);
            HOperatorSet.ImageToWorldPlane(ho_Image, out ho_ImageWorld, hv_CameraParameters, hv_CameraPoseNew2, 2000, 1500, 0.000025, "bilinear");
            //HOperatorSet.Rgb1ToGray(ho_ImageWorld, out ho_GrayImage);
            HOperatorSet.Decompose3(ho_ImageWorld, out ho_Red, out ho_Green, out ho_Blue);
            HOperatorSet.TransFromRgb(ho_Red, ho_Green, ho_Blue, out ho_Hue, out ho_Saturation, out ho_Intensity, "hsv");
            //HOperatorSet.BitNot(ho_Intensity, out ho_ImageNot);
            HOperatorSet.MeanImage(ho_Saturation, out ho_ImageMean, 10, 10);
            //HOperatorSet.SmoothImage(ho_ImageMean, out ho_ImageSmooth, "deriche2", 10);
            hv_Row = 970;
            hv_Column = 1475;
            hv_RadiusROI = 200;
            //hv_Row2 = 1170;
            //hv_Column2 = 1677;
            HOperatorSet.GenCircle(out ho_ROI, hv_Row, hv_Column, hv_RadiusROI);
            HOperatorSet.ReduceDomain(ho_ImageMean, ho_ROI, out ho_ImageROI);
            HOperatorSet.BinaryThreshold(ho_ImageROI, out ho_Region, "max_separability", "dark", out hv_UsedThreshold);
            HOperatorSet.ClosingCircle(ho_Region, out ho_RegionClosing, 50);
            HOperatorSet.Connection(ho_RegionClosing, out ho_ConnectedRegions);
            HOperatorSet.SelectShape(ho_ConnectedRegions, out ho_SelectedRegions, "area", "and", 200, 10000);
            HOperatorSet.SmallestCircle(ho_SelectedRegions, out hv_Row3, out hv_Column3, out hv_Radius);
            HOperatorSet.GetImageSize(ho_ImageWorld, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_ImageWorld, WindowControl.HalconWindow);
            HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 1);
            HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
            /*电容引脚直径不超过1.98mm,中心距离理论值10.04mm*/
            if ((int)(new HTuple((new HTuple(hv_Radius.TupleLength())).TupleEqual(2))) != 0)
            {
                HOperatorSet.GenCircle(out ho_Circle, hv_Row3, hv_Column3, hv_Radius);
                HOperatorSet.DispObj(ho_Circle, WindowControl.HalconWindow);
                if (hv_Radius[0] < 39.6 && hv_Radius[1] < 39.6)/* 39.6=1.98*0.5/0.025 */
                {
                    double i = hv_Row3.DArr[1];
                    double j = hv_Row3.DArr[0];
                    double k = hv_Column3.DArr[1];
                    double l = hv_Column3.DArr[0];
                    double a1 = (i - j) * 0.025;
                    double a2 = (k - l) * 0.025;
                    double b1 = Math.Pow(a1, 2);
                    double b2 = Math.Pow(a2, 2);
                    double c1 = b1 + b2;
                    hv_Distance = Math.Pow(c1, 0.5); ;
                    //hv_Distance = (((i - j) * 0.025) ^ 2 + ((k - l) * 0.025) ^ 2) ^ 0.5;
                    //hv_Distance = new HTuple((((hv_Row3[1] - hv_Row3[0]) * 0.025) ^ 2 + ((hv_Column3[1] - hv_Column3[0]) * 0.025) ^ 2) ^ 0.5);
                    HOperatorSet.DistancePp(hv_Row3[0], hv_Column3[0], hv_Row3[1], hv_Column3[1], out hv_Distance);
                    //if (hv_Distance < 480.8 && hv_Distance > 322.4)
                    /* 339.47=(8.06+1.98*2)/（0.025*根号2）,227.97=8.06/(0.025*根号2) */
                    if (hv_Distance < 339.47 && hv_Distance > 227.97)
                    {
                        HOperatorSet.AngleLx(hv_Row3[0], hv_Column3[0], hv_Row3[1], hv_Column3[1], out hv_Angle);
                        hv_AngleToX = (hv_Angle * 180) / 3.1415926;
                        hv_AngleToX0 = 90 - ((hv_Angle * 180) / 3.1415926);//将上照相机坐标系中电容引脚中心连线与X轴正方向的夹角变换成下照相机坐标系中的夹角
                        if (rawAngleToX - hv_AngleToX0 > 135)
                        {
                            hv_preciseAngleToX = hv_AngleToX0 + 180;
                        }
                        else if (rawAngleToX - hv_AngleToX0 < -135)
                        {
                            hv_preciseAngleToX = hv_AngleToX0 - 180;
                        }
                        else if (rawAngleToX - hv_AngleToX0 < 45 && rawAngleToX - hv_AngleToX0 > -45)
                        {
                            hv_preciseAngleToX = hv_AngleToX0;
                        }
                        else
                        {
                            double[] buf = new double[3];
                            buf[0] = 0;
                            buf[1] = 0;
                            buf[2] = 0;
                            capacityPinPosition = buf;
                            errorCode = 0505;
                            return false;
                        }
                        hv_x1 = (hv_Column3[0] + hv_Column3[1]) * 0.5 * 0.000025 - 0.025;
                        hv_y1 = (hv_Row3[0] + hv_Row3[1]) * 0.5 * 0.000025 - 0.015;
                        HOperatorSet.ReadPose(this.EyeCameraPath + poseFile2, out hv_ObjInToolPose);
                        HOperatorSet.PoseToHomMat3d(hv_ObjInToolPose, out hv_HomMat3D);
                        HOperatorSet.AffineTransPoint3d(hv_HomMat3D, hv_x1, hv_y1, 0.003, out hv_Qx, out hv_Qy, out hv_Qz);
                        double[] tuf = new double[3];
                        tuf[0] = hv_Qx * 1000;
                        tuf[1] = hv_Qy * 1000;
                        tuf[2] = hv_preciseAngleToX;
                        capacityPinPosition = tuf;
                        errorCode = 0500;
                        //HOperatorSet.DispObj(ho_Circle, WindowControl.HalconWindow);
                        return true;
                    }
                    else
                    {
                        double[] buf = new double[3];
                        buf[0] = 0;
                        buf[1] = 0;
                        buf[2] = 0;
                        capacityPinPosition = buf;
                        errorCode = 0504;
                        return false;
                    }
                }
                else
                {
                    double[] buf = new double[3];
                    buf[0] = 0;
                    buf[1] = 0;
                    buf[2] = 0;
                    capacityPinPosition = buf;
                    errorCode = 0503;
                    return false;
                }
            }
            else
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                capacityPinPosition = buf;
                errorCode = 0502;
                return false;
            }
        }
        /// <summary>
        /// 6. 焊接质量检测
        /// </summary>
        /// <param name="NGWeldPoint"></param>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：001上照相机无法获得要检测的图片
         *          002焊点检测匹配区域ROI.txt文件缺失
         *          003焊点模板图片缺失
         *          004有焊点存在质量缺陷
         */
        /*电路板离AOI底面的距离为20mm，电路板离相机感光处距离为290mm*/
        public bool AOIResult(out int[] NGWeldPoint, out int errorCode)
        {
            HObject ho_ModelImage;
            //HObject ho_ModelImageMean, ho_ModelImageSmooth;
            HObject ho_Image;
            HObject ho_ImageMean, ho_ImageSmooth;
            //HObject ho_ModelROI, ho_ModelImageROI;
            HObject ho_ShapeModelImages, ho_ShapeModelRegions;
            HObject ho_ShapeModel;
            HObject ho_ROI, ho_ImageROI;

            HTuple hv_AcqHandle;
            //HTuple hv_FileName1 = "D:/视觉系统/实际检测图片/AOI/2016-07-11_15_35_20_039.bmp";
            //HTuple hv_FileName2 = "D:/视觉系统/实际检测图片/AOI/2016-07-11_15_35_20_039 - 副本.bmp";
            //HTuple hv_CamParFile = "D:/视觉系统/shiyanchengxu/daheng/CameraParameters_down.dat";
            //HTuple hv_PoseFile1 = "D:/视觉系统/shiyanchengxu/daheng/CameraPoseNew_down.dat";
            //HTuple hv_PoseFile2 = "D:/视觉系统/shiyanchengxu/daheng/ObjInToolPose.dat";
            string shapeModel = "AOI焊点模板库";
            //string modelROI = "焊点检测模板ROI.txt";
            string regionROI = "焊点检测匹配区域ROI.txt";
            HTuple i, j;
            HTuple hv_FileHandle;
            HTuple hv_Number;
            HTuple hv_IsEOF;
            HTuple hv_Index;
            HTuple hv_R1, hv_C1, hv_R2, hv_C2;
            HTuple hv_Row1 = new HTuple(), hv_Column1 = new HTuple(), hv_Row2 = new HTuple(), hv_Column2 = new HTuple();
            //HTuple hv_R3,hv_C3,hv_R4,hv_C4;
            HTuple hv_Row3 = new HTuple(), hv_Column3 = new HTuple(), hv_Row4 = new HTuple(), hv_Column4 = new HTuple();
            HTuple hv_AreaModelRegions, hv_RowModelRegions, hv_ColumnModelRegions;
            HTuple hv_HeightPyramid;
            HTuple hv_ModelID;
            HTuple hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, hv_Score;
            HTuple hv_HeightWin, hv_WidthWin;
            HTuple buf = new HTuple();

            //读取模板图像
            //try
            //{
            //    HOperatorSet.ReadImage(out ho_ModelImage, this.EyeCameraPath + shapeModel);
            //}
            //catch
            //{
            //    int[] t = new int[1];
            //    t[0] = 0;
            //    NGWeldPoint = t;
            //    errorCode = 001;
            //    return false;
            //}
            //HOperatorSet.MeanImage(ho_ModelImage, out ho_ModelImageMean, 5, 5);
            //HOperatorSet.SmoothImage(ho_ModelImageMean, out ho_ModelImageSmooth, "deriche2", 10);
            //读取将进行焊点检测的电路板图像
            try
            {
                //HOperatorSet.ReadImage(out ho_Image, hv_FileName2);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", this.DeviceNumber, 1, -1, out hv_AcqHandle);//同时接两个相机的时候，下照相机的Device为"1"，上照相机的Device为"2"
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc3");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "image_width", 2592);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 8);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.48701);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.25174);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                int[] t = new int[1];
                t[0] = 0;
                NGWeldPoint = t;
                errorCode = 001;
                return false;
            }
            HOperatorSet.GetImageSize(ho_Image, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_Image, WindowControl.HalconWindow);
            HOperatorSet.MeanImage(ho_Image, out ho_ImageMean, 5, 5);
            HOperatorSet.SmoothImage(ho_ImageMean, out ho_ImageSmooth, "deriche2", 10);
            //读取焊点检测模板ROI.txt
            //try
            //{
            //    HOperatorSet.OpenFile(this.EyeCameraPath + modelROI, "input", out hv_FileHandle);
            //}
            //catch
            //{
            //    int[] t = new int[1];
            //    t[0] = 0;
            //    NGWeldPoint = t;
            //    errorCode = 003;
            //    return false;
            //}
            //HOperatorSet.FreadString(hv_FileHandle, out hv_Number, out hv_IsEOF);
            //hv_Number = hv_Number.TupleNumber();
            //hv_IsEOF = 0;
            //int i;
            //while (hv_IsEOF == 0)
            //{
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_Index, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_R1, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_C1, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_R2, out hv_IsEOF);
            //    HOperatorSet.FreadString(hv_FileHandle, out hv_C2, out hv_IsEOF);
            //    hv_Index = hv_Index.TupleNumber();
            //    try
            //    {
            //        i = hv_Index - 1;
            //        hv_Row1[i] = hv_R1.TupleNumber();
            //        hv_Column1[i] = hv_C1.TupleNumber();
            //        hv_Row2[i] = hv_R2.TupleNumber();
            //        hv_Column2[i] = hv_C2.TupleNumber();
            //    }
            //    catch { }
            //}
            //读取焊点检测匹配区域ROI.txt
            try
            {
                HOperatorSet.OpenFile(this.EyeCameraPath + regionROI, "input", out hv_FileHandle);
            }
            catch
            {
                int[] t = new int[1];
                t[0] = 0;
                NGWeldPoint = t;
                errorCode = 002;
                return false;
            }
            HOperatorSet.FreadString(hv_FileHandle, out hv_Number, out hv_IsEOF);
            hv_Number = hv_Number.TupleNumber();
            hv_IsEOF = 0;
            while (hv_IsEOF == 0)
            {
                HOperatorSet.FreadString(hv_FileHandle, out hv_Index, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_R1, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_C1, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_R2, out hv_IsEOF);
                HOperatorSet.FreadString(hv_FileHandle, out hv_C2, out hv_IsEOF);
                hv_Index = hv_Index.TupleNumber();
                try
                {
                    i = hv_Index - 1;
                    hv_Row1[i] = hv_R1.TupleNumber();
                    hv_Column1[i] = hv_C1.TupleNumber();
                    hv_Row2[i] = hv_R2.TupleNumber();
                    hv_Column2[i] = hv_C2.TupleNumber();
                }
                catch { }
            }
            int k = 0;
            for (i = 0; i <= 6; i = i + 1)
            {
                for (j = 0; j <= 1; j = j + 1)//7个要检测的焊点，每个焊点有2个模板
                {
                    try
                    {
                        HOperatorSet.ReadImage(out ho_ModelImage, ((this.EyeCameraPath + shapeModel + "/AOI模板" + (i.TupleString("02d"))) + (j.TupleString("02d"))) + ".bmp");
                    }
                    catch
                    {
                        int[] t = new int[1];
                        t[0] = 0;
                        NGWeldPoint = t;
                        errorCode = 003;
                        return false;
                    }
                    HOperatorSet.InspectShapeModel(ho_ModelImage, out ho_ShapeModelImages, out ho_ShapeModelRegions, 8, 30);
                    HOperatorSet.AreaCenter(ho_ShapeModelRegions, out hv_AreaModelRegions, out hv_RowModelRegions, out hv_ColumnModelRegions);
                    HOperatorSet.CountObj(ho_ShapeModelRegions, out hv_HeightPyramid);
                    HOperatorSet.CreateShapeModel(ho_ModelImage, hv_HeightPyramid, 0, (new HTuple(360)).TupleRad(), "auto", "none", "use_polarity", 30, 10, out hv_ModelID);
                    HOperatorSet.GetShapeModelContours(out ho_ShapeModel, hv_ModelID, 1);
                    HOperatorSet.GenRectangle1(out ho_ROI, hv_Row1[i], hv_Column1[i], hv_Row2[i], hv_Column2[i]);
                    HOperatorSet.ReduceDomain(ho_ImageSmooth, ho_ROI, out ho_ImageROI);
                    HOperatorSet.FindShapeModel(ho_ImageROI, hv_ModelID, 0, (new HTuple(360)).TupleRad(), 0.6, 1, 0.9, "least_squares", 0, 0.7, out hv_RowCheck, out hv_ColumnCheck, out hv_AngleCheck, out hv_Score);
                    if ((int)(new HTuple((new HTuple(hv_RowCheck.TupleLength())).TupleEqual(1))) != 0)
                    {
                        break;
                    }
                    else if (j == 1)
                    {
                        HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
                        HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
                        HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
                        HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
                        buf[k] = i + 1;
                        k = k + 1;
                    }
                }
            }
            //HOperatorSet.GenRectangle1(out ho_ModelROI, hv_Row1[i], hv_Column1[i], hv_Row2[i], hv_Column2[i]);
            //HOperatorSet.ReduceDomain(ho_ModelImageSmooth, ho_ModelROI, out ho_ModelImageROI);
            //HOperatorSet.ReadImage(out ho_ModelImage, this.EyeCameraPath + shapeModel);
            //if ((int)(new HTuple((new HTuple(hv_RowCheck.TupleLength())).TupleEqual(0))) != 0)
            //{
            //    HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            //    HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            //    HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 5);
            //    HOperatorSet.DispObj(ho_ModelROI, WindowControl.HalconWindow);
            //    buf[k]=i+1;
            //    k = k + 1;
            //}
            if (k == 0)
            {
                int[] t = new int[1];
                t[0] = 0;
                NGWeldPoint = t;
                errorCode = 000;
                return true;
            }
            else
            {
                NGWeldPoint = buf;
                errorCode = 005;
                return false;
            }
        }

        /// <summary>
        /// 7. 波峰焊接助焊剂位置检测
        /// </summary>
        /// <param name="weldingAssistantPosition"></param>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0701下照相机无法获得图像
         *          0702用hough变换无法找到波峰焊助焊剂喷头中心
         *          0703用hough变换得到的波峰焊助焊剂喷头中心位置为（0,0）
         */
        public bool WeldingAssistant(out double[] weldingAssistantPosition, out int errorCode)
        {
            HObject ho_Image;
            HObject ho_ImageWorld;
            HObject ho_Circle1, ho_Circle2, ho_Circle3, ho_Circle4, ho_Circle;
            HObject ho_ROI, ho_ROI2;
            HObject ho_ImageReduced, ho_ImageReduced2;
            HObject ho_ImageGauss;
            HObject ho_ImageEmphasize;
            HObject ho_ImaAmp, ho_ImaDir;
            HObject ho_GrayImage;
            HObject ho_Region, ho_ConnectedRegions, ho_SelectedRegions;
            HObject ho_RegionUnion;
            HObject ho_RegionClosing;
            HObject ho_Skeleton;
            HObject ho_Regionout;
            //HObject ho_ShapeModel;
            //HObject ho_ModelAtNewPosition;


            HTuple hv_Device = "";
            HTuple hv_AcqHandle;
            string camParFile = "CameraParameters_down.dat";
            string poseFile1 = "CameraPoseNew_down.dat";
            string poseFile2 = "Obj1InToolPose.dat";
            //string shapeModel = "电容粗方向检测/";
            //string modelTxt = "电容粗方向检测模板编号.txt";
            //HTuple i;
            HTuple hv_CameraParameters;
            HTuple hv_CameraPoseNew1, hv_CameraPoseNew2;
            HTuple hv_Area;
            HTuple hv_Row = new HTuple(), hv_Column = new HTuple();
            HTuple hv_x1, hv_y1;
            HTuple hv_ObjInToolPose;
            HTuple hv_HomMat3D;
            HTuple hv_Qx, hv_Qy, hv_Qz;
            HTuple hv_HeightWin, hv_WidthWin;
            //HTuple hv_FileHandle;
            //HTuple hv_MN, hv_IsEOF;
            //HTuple hv_ModelName = new HTuple();
            //HTuple hv_ModelID;
            //HTuple hv_RowCheck, hv_ColumnCheck, hv_AngleCheck, hv_Score;
            //HTuple hv_MovementOfObject;
            /*这些参数调试时可能要修改*/
            HTuple hv_Row0 = 830;
            HTuple hv_Column0 = 817;
            /*
             * 利用skeleton算法与hough变换计算电容中心位置
             */
            if (this.DeviceNumber == "1")
            {
                hv_Device = "2";
            }
            else if (this.DeviceNumber == "2")
            {
                hv_Device = "1";
            }
            try
            {
                //HOperatorSet.ReadImage(out ho_Image, hv_FileName);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", hv_Device, 1, -1, out hv_AcqHandle);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc1");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.01);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.42);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                weldingAssistantPosition = buf;
                errorCode = 0701;
                return false;
            }
            HOperatorSet.ReadCamPar(this.HandCameraPath + camParFile, out hv_CameraParameters);
            HOperatorSet.ReadPose(this.HandCameraPath + poseFile1, out hv_CameraPoseNew1);
            HOperatorSet.SetOriginPose(hv_CameraPoseNew1, -0.025, -0.016, 0, out hv_CameraPoseNew2);
            HOperatorSet.ImageToWorldPlane(ho_Image, out ho_ImageWorld, hv_CameraParameters, hv_CameraPoseNew2, 2000, 1500, 0.000025, "bilinear");
            //HOperatorSet.Rgb1ToGray(ho_ImageWorld, out ho_GrayImage);
            HOperatorSet.GetImageSize(ho_ImageWorld, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_ImageWorld, WindowControl.HalconWindow);
            HOperatorSet.GenCircle(out ho_Circle1, hv_Row0, hv_Column0, 700);//调试时可能要修改半径
            HOperatorSet.GenCircle(out ho_Circle2, hv_Row0, hv_Column0, 400);
            HOperatorSet.Difference(ho_Circle1, ho_Circle2, out ho_ROI);
            HOperatorSet.ReduceDomain(ho_ImageWorld, ho_ROI, out ho_ImageReduced);
            HOperatorSet.GaussFilter(ho_ImageReduced, out ho_ImageGauss, 7);
            HOperatorSet.Emphasize(ho_ImageGauss, out ho_ImageEmphasize, 7, 7, 2);
            HOperatorSet.EdgesColor(ho_ImageEmphasize, out ho_ImaAmp, out ho_ImaDir, "canny", 1, "nms", 20, 40);
            HOperatorSet.GenCircle(out ho_Circle3, hv_Row0, hv_Column0, 670);//调试时可能要修改半径
            HOperatorSet.GenCircle(out ho_Circle4, hv_Row0, hv_Column0, 420);
            HOperatorSet.Difference(ho_Circle3, ho_Circle4, out ho_ROI2);
            HOperatorSet.ReduceDomain(ho_ImaAmp, ho_ROI2, out ho_ImageReduced2);
            HOperatorSet.Rgb1ToGray(ho_ImageReduced2, out ho_GrayImage);
            HOperatorSet.Threshold(ho_GrayImage, out ho_Region, 30, 255);//调试时可能要修改灰度的阈值
            HOperatorSet.Connection(ho_Region, out ho_ConnectedRegions);
            HOperatorSet.SelectShape(ho_ConnectedRegions, out ho_SelectedRegions, "area", "and", 100, 99999999);//调试时可能要修改面积选择范围
            HOperatorSet.Union1(ho_SelectedRegions, out ho_RegionUnion);
            HOperatorSet.ClosingCircle(ho_RegionUnion, out ho_RegionClosing, 15);
            HOperatorSet.Skeleton(ho_RegionClosing, out ho_Skeleton);
            HOperatorSet.HoughCircles(ho_Skeleton, out ho_Regionout, 560, 7, 2);//调试时可能要修改半径
            HOperatorSet.AreaCenter(ho_Regionout, out hv_Area, out hv_Row, out hv_Column);
            try
            {
                HOperatorSet.GenCircle(out ho_Circle, hv_Row[0], hv_Column[0], 565);
            }
            catch
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                weldingAssistantPosition = buf;
                errorCode = 0702;
                return false;
            }
            HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 1);
            HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
            HOperatorSet.DispObj(ho_Circle, WindowControl.HalconWindow);
            if ((int)((new HTuple(((hv_Row.TupleSelect(0))).TupleEqual(0))).TupleAnd(new HTuple(((hv_Column.TupleSelect(0))).TupleEqual(0)))) != 0)
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                weldingAssistantPosition = buf;
                errorCode = 0703;
                return false;
            }
            hv_x1 = (hv_Column.TupleSelect(0) * 0.000025) - 0.025;
            hv_y1 = (hv_Row.TupleSelect(0) * 0.000025) - 0.016;
            HOperatorSet.ReadPose(this.HandCameraPath + poseFile2, out hv_ObjInToolPose);
            HOperatorSet.PoseToHomMat3d(hv_ObjInToolPose, out hv_HomMat3D);
            HOperatorSet.AffineTransPoint3d(hv_HomMat3D, hv_x1, hv_y1, 0.003, out hv_Qx, out hv_Qy, out hv_Qz);
            double[] tuf = new double[3];
            tuf[0] = hv_Qx * 1000;
            tuf[1] = hv_Qy * 1000;
            tuf[2] = hv_Qz * 1000;
            weldingAssistantPosition = tuf;
            errorCode = 0700;
            return true;
        }

        /// <summary>
        /// 8. 波峰焊接焊锡喷头位置检测
        /// </summary>
        /// <param name="weldingTinPosition"></param>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0801下照相机无法获得图像
         *          0802用hough变换无法找到焊锡喷头中心
         *          0803用hough变换得到的焊锡喷头中心位置为（0,0）
         */
        public bool WeldingTin(out double[] weldingTinPosition, out int errorCode)
        {
            HObject ho_Image;
            HObject ho_ImageWorld;
            HObject ho_Circle1, ho_Circle2, ho_Circle3, ho_Circle4, ho_Circle;
            HObject ho_ROI, ho_ROI2;
            HObject ho_ImageReduced, ho_ImageReduced2;
            HObject ho_ImageGauss;
            HObject ho_ImageEmphasize;
            HObject ho_ImaAmp, ho_ImaDir;
            HObject ho_GrayImage;
            HObject ho_Region, ho_ConnectedRegions, ho_SelectedRegions;
            HObject ho_RegionUnion;
            HObject ho_RegionClosing;
            HObject ho_Skeleton;
            HObject ho_Regionout;


            HTuple hv_Device = "";
            HTuple hv_AcqHandle;
            string camParFile = "CameraParameters_down.dat";
            string poseFile1 = "CameraPoseNew_down.dat";
            string poseFile2 = "Obj1InToolPose.dat";
            HTuple hv_CameraParameters;
            HTuple hv_CameraPoseNew1, hv_CameraPoseNew2;
            HTuple hv_Area;
            HTuple hv_Row = new HTuple(), hv_Column = new HTuple();
            HTuple hv_x1, hv_y1;
            HTuple hv_ObjInToolPose;
            HTuple hv_HomMat3D;
            HTuple hv_Qx, hv_Qy, hv_Qz;
            HTuple hv_HeightWin, hv_WidthWin;
            /*这些参数调试时可能要修改*/
            HTuple hv_Row0 = 830;
            HTuple hv_Column0 = 817;
            /*
             * 利用skeleton算法与hough变换计算电容中心位置
             */
            if (this.DeviceNumber == "1")
            {
                hv_Device = "2";
            }
            else if (this.DeviceNumber == "2")
            {
                hv_Device = "1";
            }
            try
            {
                //HOperatorSet.ReadImage(out ho_Image, hv_FileName);
                HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", hv_Device, 1, -1, out hv_AcqHandle);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "adc_level", "adc1");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 2);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_b", 1.01);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_g", 1.0);
                HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance_r", 1.42);
                HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
            }
            catch
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                weldingTinPosition = buf;
                errorCode = 0801;
                return false;
            }
            HOperatorSet.ReadCamPar(this.HandCameraPath + camParFile, out hv_CameraParameters);
            HOperatorSet.ReadPose(this.HandCameraPath + poseFile1, out hv_CameraPoseNew1);
            HOperatorSet.SetOriginPose(hv_CameraPoseNew1, -0.025, -0.016, 0, out hv_CameraPoseNew2);
            HOperatorSet.ImageToWorldPlane(ho_Image, out ho_ImageWorld, hv_CameraParameters, hv_CameraPoseNew2, 2000, 1500, 0.000025, "bilinear");
            //HOperatorSet.Rgb1ToGray(ho_ImageWorld, out ho_GrayImage);
            HOperatorSet.GetImageSize(ho_ImageWorld, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
            HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
            HOperatorSet.DispObj(ho_ImageWorld, WindowControl.HalconWindow);
            HOperatorSet.GenCircle(out ho_Circle1, hv_Row0, hv_Column0, 700);//调试时可能要修改半径
            HOperatorSet.GenCircle(out ho_Circle2, hv_Row0, hv_Column0, 400);
            HOperatorSet.Difference(ho_Circle1, ho_Circle2, out ho_ROI);
            HOperatorSet.ReduceDomain(ho_ImageWorld, ho_ROI, out ho_ImageReduced);
            HOperatorSet.GaussFilter(ho_ImageReduced, out ho_ImageGauss, 7);
            HOperatorSet.Emphasize(ho_ImageGauss, out ho_ImageEmphasize, 7, 7, 2);
            HOperatorSet.EdgesColor(ho_ImageEmphasize, out ho_ImaAmp, out ho_ImaDir, "canny", 1, "nms", 20, 40);
            HOperatorSet.GenCircle(out ho_Circle3, hv_Row0, hv_Column0, 670);//调试时可能要修改半径
            HOperatorSet.GenCircle(out ho_Circle4, hv_Row0, hv_Column0, 420);
            HOperatorSet.Difference(ho_Circle3, ho_Circle4, out ho_ROI2);
            HOperatorSet.ReduceDomain(ho_ImaAmp, ho_ROI2, out ho_ImageReduced2);
            HOperatorSet.Rgb1ToGray(ho_ImageReduced2, out ho_GrayImage);
            HOperatorSet.Threshold(ho_GrayImage, out ho_Region, 30, 255);//调试时可能要修改灰度的阈值
            HOperatorSet.Connection(ho_Region, out ho_ConnectedRegions);
            HOperatorSet.SelectShape(ho_ConnectedRegions, out ho_SelectedRegions, "area", "and", 100, 99999999);//调试时可能要修改面积选择范围
            HOperatorSet.Union1(ho_SelectedRegions, out ho_RegionUnion);
            HOperatorSet.ClosingCircle(ho_RegionUnion, out ho_RegionClosing, 15);
            HOperatorSet.Skeleton(ho_RegionClosing, out ho_Skeleton);
            HOperatorSet.HoughCircles(ho_Skeleton, out ho_Regionout, 560, 7, 2);//调试时可能要修改半径
            HOperatorSet.AreaCenter(ho_Regionout, out hv_Area, out hv_Row, out hv_Column);
            try
            {
                HOperatorSet.GenCircle(out ho_Circle, hv_Row[0], hv_Column[0], 565);
            }
            catch
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                weldingTinPosition = buf;
                errorCode = 0802;
                return false;
            }
            HOperatorSet.SetColor(WindowControl.HalconWindow, "red");
            HOperatorSet.SetDraw(WindowControl.HalconWindow, "margin");
            HOperatorSet.SetLineWidth(WindowControl.HalconWindow, 1);
            HOperatorSet.DispObj(ho_ROI, WindowControl.HalconWindow);
            HOperatorSet.DispObj(ho_Circle, WindowControl.HalconWindow);
            if ((int)((new HTuple(((hv_Row.TupleSelect(0))).TupleEqual(0))).TupleAnd(new HTuple(((hv_Column.TupleSelect(0))).TupleEqual(0)))) != 0)
            {
                double[] buf = new double[3];
                buf[0] = 0;
                buf[1] = 0;
                buf[2] = 0;
                weldingTinPosition = buf;
                errorCode = 0803;
                return false;
            }
            hv_x1 = (hv_Column.TupleSelect(0) * 0.000025) - 0.025;
            hv_y1 = (hv_Row.TupleSelect(0) * 0.000025) - 0.016;
            HOperatorSet.ReadPose(this.HandCameraPath + poseFile2, out hv_ObjInToolPose);
            HOperatorSet.PoseToHomMat3d(hv_ObjInToolPose, out hv_HomMat3D);
            HOperatorSet.AffineTransPoint3d(hv_HomMat3D, hv_x1, hv_y1, 0.003, out hv_Qx, out hv_Qy, out hv_Qz);
            double[] tuf = new double[3];
            tuf[0] = hv_Qx * 1000;
            tuf[1] = hv_Qy * 1000;
            tuf[2] = hv_Qz * 1000;
            weldingTinPosition = tuf;
            errorCode = 0800;
            return true;
        }

        /// <summary>
        /// 下照相机手眼标定
        /// </summary>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：0901描述标定板的descr文件缺失
         *          0902缺少标定所需的图片
         *          0903包含拍摄这张图像时的ToolInBasePose的dat文件缺失
         *          0904写入相机内参、位姿等文件失败
         */
        /*试验时观察平面距相机镜头前的距离为360mm，实际工作时应为300mm*/
        public bool HandCameraCalibration(out int errorCode)
        {

            HObject ho_Image;

            HTuple hv_CalibDataID;
            HTuple hv_StartCamParam = new HTuple();
            //HTuple hv_CalTabFile = "C:/Users/Public/Documents/MVTec/HALCON-12.0/examples/3d_models/universal_joint_part/caltab_20mm.descr";
            string calTabFile = "caltab_20mm.descr";
            string calibImage = "标定/";
            HTuple i;
            HTuple hv_ToolInBasePose;
            HTuple hv_TmpCtrl_FindCalObjParNames = new HTuple();
            hv_TmpCtrl_FindCalObjParNames[0] = "gap_tolerance";
            hv_TmpCtrl_FindCalObjParNames[1] = "alpha";
            hv_TmpCtrl_FindCalObjParNames[2] = "max_diam_marks";
            hv_TmpCtrl_FindCalObjParNames[3] = "skip_find_caltab";
            HTuple hv_TmpCtrl_FindCalObjParValues = new HTuple();
            hv_TmpCtrl_FindCalObjParValues[0] = 1;
            hv_TmpCtrl_FindCalObjParValues[1] = 1;
            hv_TmpCtrl_FindCalObjParValues[2] = 100;
            hv_TmpCtrl_FindCalObjParValues[3] = "false";
            HTuple hv_Errors;
            HTuple hv_CameraParameters;
            HTuple hv_ToolInCamPose, hv_CalObjInBasePose;
            HTuple hv_BaseInToolPose;
            HTuple hv_Obj1InToolPose;
            HTuple hv_Obj1InCamPose;
            HTuple hv_CameraPoseNew1;


            //try
            //{
            //    HOperatorSet.ReadImage(out ho_Image, this.HandCameraPath + calibImage + "calib_00.jpg");
            //}
            //catch
            //{
            //    errorCode = 0901;
            //    return false;
            //}
            HOperatorSet.CreateCalibData("hand_eye_moving_cam", 1, 1, out hv_CalibDataID);
            hv_StartCamParam[0] = 0.035;
            hv_StartCamParam[1] = 0;
            hv_StartCamParam[2] = 2.2e-006;
            hv_StartCamParam[3] = 2.2e-006;
            hv_StartCamParam[4] = 1296;
            hv_StartCamParam[5] = 972;
            hv_StartCamParam[6] = 2592;
            hv_StartCamParam[7] = 1944;
            HOperatorSet.SetCalibDataCamParam(hv_CalibDataID, 0, "area_scan_division", hv_StartCamParam);
            try
            {
                HOperatorSet.SetCalibDataCalibObject(hv_CalibDataID, 0, this.HandCameraPath + calibImage + calTabFile);
            }
            catch
            {
                errorCode = 0901;
                return false;
            }
            HOperatorSet.SetCalibData(hv_CalibDataID, "model", "general", "optimization_method", "nonlinear");
            for (i = 0; i < 14; i = i + 1)
            {
                try
                {
                    HOperatorSet.ReadImage(out ho_Image, this.HandCameraPath + calibImage + "calib_" + (i.TupleString("02d")) + ".bmp");
                }
                catch
                {
                    errorCode = 0902;
                    return false;
                }
                //HOperatorSet.FindCalibObject(ho_Image, hv_CalibDataID, 0, 0, i, new HTuple(), new HTuple());
                HOperatorSet.FindCalibObject(ho_Image, hv_CalibDataID, 0, 0, i, hv_TmpCtrl_FindCalObjParNames, hv_TmpCtrl_FindCalObjParValues);
                try
                {
                    HOperatorSet.ReadPose(this.HandCameraPath + calibImage + "movingcam_robot_pose_" + (i.TupleString("02d")) + ".dat", out hv_ToolInBasePose);
                }
                catch
                {
                    errorCode = 0903;
                    return false;
                }
                HOperatorSet.SetCalibData(hv_CalibDataID, "tool", i, "tool_in_base_pose", hv_ToolInBasePose);
            }
            HOperatorSet.CalibrateHandEye(hv_CalibDataID, out hv_Errors);
            HOperatorSet.GetCalibData(hv_CalibDataID, "camera", 0, "params", out hv_CameraParameters);
            HOperatorSet.GetCalibData(hv_CalibDataID, "camera", 0, "tool_in_cam_pose", out hv_ToolInCamPose);
            HOperatorSet.GetCalibData(hv_CalibDataID, "calib_obj", 0, "obj_in_base_pose", out hv_CalObjInBasePose);
            HOperatorSet.ReadPose(this.HandCameraPath + calibImage + "movingcam_robot_pose_00.dat", out hv_ToolInBasePose);
            HOperatorSet.PoseInvert(hv_ToolInBasePose, out hv_BaseInToolPose);
            HOperatorSet.PoseCompose(hv_BaseInToolPose, hv_CalObjInBasePose, out hv_Obj1InToolPose);
            HOperatorSet.PoseCompose(hv_ToolInCamPose, hv_Obj1InToolPose, out hv_Obj1InCamPose);
            HOperatorSet.SetOriginPose(hv_Obj1InCamPose, 0, 0, 0.003, out hv_CameraPoseNew1);
            try
            {
                HOperatorSet.WriteCamPar(hv_CameraParameters, this.HandCameraPath + "CameraParameters1_down.dat");
                HOperatorSet.WritePose(hv_CameraPoseNew1, this.HandCameraPath + "CameraPoseNew1_down.dat");
                HOperatorSet.WritePose(hv_Obj1InToolPose, this.HandCameraPath + "Obj1InToolPose1.dat");
                HOperatorSet.WritePose(hv_CalObjInBasePose, this.HandCameraPath + "Obj1InBasePose1.dat");
                errorCode = 0900;
                return true;
            }
            catch
            {
                errorCode = 0904;
                return false;
            }
        }
        /// <summary>
        /// 上照相机手眼标定
        /// </summary>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        /*故障代码：1001描述标定板的descr文件缺失
         *          1002缺少标定所需的图片
         *          1003包含拍摄这张图像时的ToolInBasePose的dat文件缺失
         *          1004写入相机内参、位姿等文件失败
         */
        /*观察平面距相机镜头前的距离为250mm*/
        public bool EyeCameraCalibration(out int errorCode)
        {
            HObject ho_Image;

            HTuple hv_CalibDataID;
            HTuple hv_StartCamParam = new HTuple();
            //HTuple hv_CalTabFile = "C:/Users/Public/Documents/MVTec/HALCON-12.0/examples/3d_models/universal_joint_part/caltab_20mm.descr";
            string calTabFile = "caltab_20mm.descr";
            string calibImage = "标定/";
            HTuple i;
            HTuple hv_ToolInBasePose;
            HTuple hv_Errors;
            HTuple hv_CameraParameters;
            HTuple hv_BaseInCamPose;
            HTuple hv_Obj2InBasePose;
            HTuple hv_Obj2InToolPose;
            HTuple hv_Obj2InCamPose;
            HTuple hv_CameraPoseNew2;


            //try
            //{
            //    HOperatorSet.ReadImage(out ho_Image, this.EyeCameraPath + calibImage + "calib_00.jpg");
            //}
            //catch
            //{
            //    errorCode = 1001;
            //    return false;
            //}
            HOperatorSet.CreateCalibData("hand_eye_stationary_cam", 1, 1, out hv_CalibDataID);
            hv_StartCamParam[0] = 0.016;
            hv_StartCamParam[1] = 0;
            hv_StartCamParam[2] = 2.2e-006;
            hv_StartCamParam[3] = 2.2e-006;
            hv_StartCamParam[4] = 1296;
            hv_StartCamParam[5] = 972;
            hv_StartCamParam[6] = 2592;
            hv_StartCamParam[7] = 1944;
            HOperatorSet.SetCalibDataCamParam(hv_CalibDataID, 0, "area_scan_division", hv_StartCamParam);
            try
            {
                HOperatorSet.SetCalibDataCalibObject(hv_CalibDataID, 0, this.EyeCameraPath + calibImage + calTabFile);
            }
            catch
            {
                errorCode = 1001;
                return false;
            }
            HOperatorSet.SetCalibData(hv_CalibDataID, "model", "general", "optimization_method", "nonlinear");
            for (i = 0; i < 17; i = i + 1)
            {
                try
                {
                    HOperatorSet.ReadImage(out ho_Image, this.EyeCameraPath + calibImage + "calib_" + (i.TupleString("02d")) + ".bmp");
                }
                catch
                {
                    errorCode = 1002;
                    return false;
                }
                HOperatorSet.FindCalibObject(ho_Image, hv_CalibDataID, 0, 0, i, new HTuple(), new HTuple());
                try
                {
                    HOperatorSet.ReadPose(this.EyeCameraPath + calibImage + "stationarycam_robot_pose_" + (i.TupleString("02d")) + ".dat", out hv_ToolInBasePose);
                }
                catch
                {
                    errorCode = 1003;
                    return false;
                }
                HOperatorSet.SetCalibData(hv_CalibDataID, "tool", i, "tool_in_base_pose", hv_ToolInBasePose);
            }
            HOperatorSet.CalibrateHandEye(hv_CalibDataID, out hv_Errors);
            HOperatorSet.GetCalibData(hv_CalibDataID, "camera", 0, "params", out hv_CameraParameters);
            HOperatorSet.GetCalibData(hv_CalibDataID, "camera", 0, "base_in_cam_pose", out hv_BaseInCamPose);
            HOperatorSet.GetCalibData(hv_CalibDataID, "calib_obj", 0, "obj_in_tool_pose", out hv_Obj2InToolPose);
            HOperatorSet.ReadPose(this.EyeCameraPath + calibImage + "stationarycam_robot_pose_00.dat", out hv_ToolInBasePose);
            HOperatorSet.PoseCompose(hv_ToolInBasePose, hv_Obj2InToolPose, out hv_Obj2InBasePose);
            HOperatorSet.PoseCompose(hv_BaseInCamPose, hv_Obj2InBasePose, out hv_Obj2InCamPose);
            HOperatorSet.SetOriginPose(hv_Obj2InCamPose, 0, 0, 0.003, out hv_CameraPoseNew2);
            try
            {
                HOperatorSet.WriteCamPar(hv_CameraParameters, this.EyeCameraPath + "CameraParameters_up1.dat");
                HOperatorSet.WritePose(hv_CameraPoseNew2, this.EyeCameraPath + "CameraPoseNew_up1.dat");
                HOperatorSet.WritePose(hv_Obj2InToolPose, this.EyeCameraPath + "Obj2InToolPose1.dat");
                errorCode = 1000;
                return true;
            }
            catch
            {
                errorCode = 1004;
                return false;
            }
        }
        //public TextBox tb;
        /// <summary>
        /// 通过halcon控制相机获取图像
        /// </summary>
        /// <param name="errorCode"></param>
        /// <returns></returns>
        public bool AcquireImage(out int errorCode)
        {
            HObject ho_Image;
            HTuple hv_AcqHandle;
            HTuple hv_HeightWin, hv_WidthWin;


            HOperatorSet.OpenFramegrabber("DahengCAM", 1, 1, 0, 0, 0, 0, "interlaced", 8, "rgb", -1, "false", "HV-xx51", "1", 1, -1, out hv_AcqHandle);
            HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "shutter", 30);
            HOperatorSet.SetFramegrabberParam(hv_AcqHandle, "white_balance", "enable");
            HOperatorSet.GrabImageStart(hv_AcqHandle, -1);
            try
            {
                HOperatorSet.GrabImageAsync(out ho_Image, hv_AcqHandle, -1);
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
                HOperatorSet.GetImageSize(ho_Image, out hv_HeightWin, out hv_WidthWin);// 获取输入图像的尺寸
                HOperatorSet.SetPart(WindowControl.HalconWindow, 0, 0, hv_WidthWin, hv_HeightWin);//将获得的图像铺满整个窗口
                HOperatorSet.DispObj(ho_Image, WindowControl.HalconWindow);
                errorCode = 000;
                return true;
            }
            catch
            {
                HOperatorSet.CloseFramegrabber(hv_AcqHandle);
                errorCode = 001;
                return false;
            }
        }

        //public void Show(IBaseData objIBaseData)
        //{
        //    GX_VALID_BIT_LIST emValidBits = GX_VALID_BIT_LIST.GX_BIT_0_7;

        //    //检查图像是否改变并更新Buffer
        //    __UpdateBufferSize(objIBaseData);


        //    if (null != objIBaseData)
        //    {
        //        emValidBits = __GetBestValudBit(objIBaseData.GetPixelFormat());
        //        if (GX_FRAME_STATUS_LIST.GX_FRAME_STATUS_SUCCESS == objIBaseData.GetStatus())
        //        {
        //            if (m_bIsColor)
        //            {
        //                IntPtr pBufferColor = objIBaseData.ConvertToRGB24(emValidBits, GX_BAYER_CONVERT_TYPE_LIST.GX_RAW2RGB_NEIGHBOUR, false);
        //                Marshal.Copy(pBufferColor, m_byColorBuffer, 0, __GetStride(m_nWidth, m_bIsColor) * m_nHeigh);
        //                __ShowImage(m_byColorBuffer);
        //            }
        //            else
        //            {
        //                IntPtr pBufferMono = IntPtr.Zero;
        //                if (__IsPixelFormat8(objIBaseData.GetPixelFormat()))
        //                {
        //                    pBufferMono = objIBaseData.GetBuffer();
        //                }
        //                else
        //                {
        //                    pBufferMono = objIBaseData.ConvertToRaw8(emValidBits);
        //                }
        //                Marshal.Copy(pBufferMono, m_byMonoBuffer, 0, __GetStride(m_nWidth, m_bIsColor) * m_nHeigh);
        //                __ShowImage(m_byMonoBuffer);
        //            }
        //        }
        //    }
        //}
        //public bool AcquireImage2(out int errorCode)
        //{

        //    IGXDevice m_objIGXDevice = null;
        //    PictureBox m_pic_ShowImage = null;
        //    GxIAPINET.Sample.Common.GxBitmap gxBitmap = new GxIAPINET.Sample.Common.GxBitmap(m_objIGXDevice, m_pic_ShowImage);
        //    GxIAPINET.Sample.Common.GxBitmap.__CreateBitmap(out m_bitmap, m_nWidth, m_nHeigh, m_bIsColor);

        //    IGXDevice objIGXDevice = null;

        //    IGXStream objIGXStream = objIGXDevice.OpenStream(0);
        //    //开启流通道采集
        //    objIGXStream.StartGrab();
        //    //给设备发送开采命令
        //    IGXFeatureControl objIGXFeatureControl = objIGXDevice.GetRemoteFeatureControl();
        //    objIGXFeatureControl.GetCommandFeature("AcquisitionStart").Execute();
        //    //采单帧
        //    IImageData objImageData = null;
        //    objImageData = objIGXStream.GetImage(500);//超时时间使用500ms，用户可以自行设定
        //    if (objImageData.GetStatus() == GX_FRAME_STATUS_LIST.GX_FRAME_STATUS_SUCCESS)
        //    {
        //        //采图成功而且是完整帧，可以进行图像处理...
        //    }
        //    objImageData.Destroy();//销毁objImageData对象
        //    //停采
        //    objIGXFeatureControl.GetCommandFeature("AcquisitionStop").Execute();
        //    objIGXStream.StopGrab();
        //    //关闭流通道
        //    objIGXStream.Close();

        //    HOperatorSet.GenImageInterleaved(out ho_Image, HTuple pixelPointer, HTuple colorFormat, HTuple originalWidth, HTuple originalHeight, HTuple alignment, HTuple type, HTuple imageWidth, HTuple imageHeight, HTuple startRow, HTuple startColumn, HTuple bitsPerChannel, HTuple bitShift)

        //    errorCode = 000;
        //    return true;

        //}
    }
}
