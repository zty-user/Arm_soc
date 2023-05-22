//////////////////////////////////////////////////////////////////
//  复旦大学2019-2020学年秋季学期“系统级可编程芯片设计”课程期末Project
//  无人值守停车场的收费关卡系统
//  Team : 任予琛 + 靳子路 + 刘思敏
//  Written by : 靳子路
//  Date : 2019.12
//////////////////////////////////////////////////////////////////

#include "plr.h"

using namespace cv; 
using namespace std;

//车牌区域范围判断
bool verifySizes_closeImg(const RotatedRect & candidate)
{
    float error = 0.4;//误差40%
    const float aspect = 4.7272;//44/14;//长宽比
    int min = 25*aspect*25;//面积下限
    int max = 180*aspect*180;//面积上限
    float rmin = aspect - aspect*error; //考虑误差后的最小长宽比
    float rmax = aspect + aspect*error; //考虑误差后的最大长宽比

    int area = candidate.size.height * candidate.size.width;//计算面积
    float r = (float)candidate.size.width/(float)candidate.size.height;//计算宽高比
    if(r <1)
        r = 1/r;

    if( (area < min || area > max) || (r< rmin || r > rmax)  )//满足条件才认为是车牌候选区域
        return false;
    else
        return true;
}

void RgbConvToGray(const Mat& inputImage,Mat & outpuImage)
{
    outpuImage = Mat(inputImage.rows ,inputImage.cols ,CV_8UC1);  

    for (int i = 0 ;i<inputImage.rows ;++ i)
    {
        uchar *ptrGray = outpuImage.ptr<uchar>(i); 
        const Vec3b * ptrRgb = inputImage.ptr<Vec3b>(i);
        for (int j = 0 ;j<inputImage.cols ;++ j)
        {
            ptrGray[j] = 0.3*ptrRgb[j][2]+0.59*ptrRgb[j][1]+0.11*ptrRgb[j][0];  
        }
    }
}
//获得144*33的候选车牌区域
void normal_area(Mat &intputImg, vector<RotatedRect> &rects_optimal, vector <Mat> &output_area)
{
    float r, angle;
    for (int i = 0; i< rects_optimal.size(); ++i)
    {
        //旋转区域
        angle = rects_optimal[i].angle;
        r = (float)rects_optimal[i].size.width / (float) (float)rects_optimal[i].size.height;
        if(r<1)
            angle = 90 + angle;//旋转图像使其得到长大于高度图像。
        Mat rotmat = getRotationMatrix2D(rects_optimal[i].center, angle,1);//对矩阵图像旋转
        Mat img_rotated;
        warpAffine(intputImg, img_rotated, rotmat, intputImg.size(), CV_INTER_CUBIC);//对旋转后的图形映射处理
        Size rect_size = rects_optimal[i].size;
        if(r<1)
            swap(rect_size.width, rect_size.height);//交换高和宽
        Mat  img_crop;
        getRectSubPix(img_rotated, rect_size, rects_optimal[i].center, img_crop);//图像切割

        //用光照直方图调整所有裁剪得到的图像，使具有相同宽度和高度，适用于训练和分类
        Mat resultResized;
        resultResized.create(33,144,CV_8UC3);//CV32FC1？？？？
        resize(img_crop, resultResized,resultResized.size(), 0, 0, INTER_CUBIC);
        Mat grayResult;
        RgbConvToGray(resultResized, grayResult);
        blur(grayResult, grayResult, Size(3,3));//用标准化的盒式过滤器来平滑图像，可以不用
        equalizeHist(grayResult, grayResult);//直方图均衡化，用于归一化图像亮度和增强对比度
        output_area.push_back(grayResult);
    }
}
//判断是非为车牌字符
bool char_verifySizes(const RotatedRect & candidate)
{
	float aspect = 45.0f/77.0f;//45.0f/90.0f;
	float width,height;
	//因为是RotatedRect类型，所以需要确保高比宽长
	if (candidate.size.width >=candidate.size.height)
	{
		width = (float) candidate.size.height;
		height = (float) candidate.size.width;
	}
	else 
	{
		width = (float) candidate.size.width;
		height  = (float)candidate.size.height;
	}

	float charAspect = (float) width/ (float)height;//宽高比

	float error = 0.5f;
	float minHeight = 15.0f;
	float maxHeight = 35.0f;

	float minAspect = 0.1f;//考虑到数字1，所以最小比率取0.1
	float maxAspect = aspect + aspect * error;

	if( charAspect > minAspect && charAspect <= maxAspect
		&&  height>= minHeight && height< maxHeight)
		return true;
	else
		return false;
}
//取前7个字符，对字符区域进行排序
void char_sort(vector <RotatedRect > & in_char )
{
	vector <RotatedRect >  out_char;
	const int length = 7;
	int index[length] = {0,1,2,3,4,5,6};
	float centerX[length];
	for (int i=0; i < length; ++i)
	{
		centerX[i] = in_char[i].center.x;
	}

	for (int j=0; j < length; j++) {
		for (int i = length-2; i >= j;i--)
			if (centerX[i] > centerX[i+1])
			{
				float t=centerX[i];
				centerX[i]=centerX[i+1];
				centerX[i+1]=t;

				int tt = index[i];
				index[i] = index[i+1];
				index[i+1] = tt;
			}
	}

	for(int i=0;i<length ;i++)
		out_char.push_back(in_char[(index[i])]);

	in_char.clear();
	in_char = out_char;
}
//处理车牌上的铆钉,依次扫描各行，判断跳变的次数，字符所在的行跳变次数较多，铆钉所在行则较少
bool clearMaoDing(Mat &img) {
  	std::vector<float> fJump;
  	int whiteCount = 0;
  	const int x = 7;
  	Mat jump = Mat::zeros(1, img.rows, CV_32F);
  	for (int i = 0; i < img.rows; i++) {
    	int jumpCount = 0;//跳变次数
    	for (int j = 0; j < img.cols - 1; j++) {
      		if (img.at<char>(i, j) != img.at<char>(i, j + 1)) 
      			jumpCount++;
      		if (img.at<uchar>(i, j) == 255) {
        		whiteCount++;
      		}
    	}
    jump.at<float>(i) = (float) jumpCount;
  	}

  	int iCount = 0;//
 	for (int i = 0; i < img.rows; i++) {
    	fJump.push_back(jump.at<float>(i));
    	if (jump.at<float>(i) >= 16 && jump.at<float>(i) <= 45) {
      		iCount++;
    	}
  	}

  	//如果
  	if (iCount * 1.0 / img.rows <= 0.40) {
    	return false;
  	}
  	//如果白点太少或太多，则判断为不是车牌
  	if (whiteCount * 1.0 / (img.rows * img.cols) < 0.15 ||
      	whiteCount * 1.0 / (img.rows * img.cols) > 0.50) {
    		return false;
  	}
  	//清除行上的铆钉
  	for (int i = 0; i < img.rows; i++) {
    	if (jump.at<float>(i) <= x) {
      		for (int j = 0; j < img.cols; j++) {
        		img.at<char>(i, j) = 0;
      		}
    	}
  	}
  	return true;
}

//判断是否存在车牌
bool is_plate(Mat& inputImg)
{
	Mat img_th;
	threshold(inputImg, img_th, 180, 255, CV_THRESH_BINARY);
	if (!clearMaoDing(img_th)){
		return false;
	}
	img_th.copyTo(inputImg);
	return true;
}

//得到20*20的标准字符分割图像
bool char_segment(const Mat& inputImg,vector <Mat>& dst_mat)
{
	Mat img_threshold;
	Mat element = getStructuringElement(MORPH_RECT ,Size(3 ,3));//闭形态学的结构元素
	morphologyEx(inputImg ,inputImg,CV_MOP_CLOSE,element);//形态学处理
	Mat img_contours;
	inputImg.copyTo(img_contours);
	Mat result2;

	vector<vector<Point>> contours;
	//提取目标轮廓，输出每一个连通区域的轮廓点的集合
	findContours(img_contours, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	vector<vector<Point>> :: iterator itc = contours.begin();
	vector<RotatedRect> char_rects;

	while( itc != contours.end())
	{
		RotatedRect minArea = minAreaRect(Mat(*itc));//返回每个轮廓的最小有界矩形区域
		Point2f vertices[4];
		minArea.points(vertices);

		if(!char_verifySizes(minArea))//判断矩形轮廓是否符合要求
		{
			itc = contours.erase(itc);
		}
		else     
		{
			++itc; 
			char_rects.push_back(minArea);
		}  	
	}
	char_sort(char_rects);//对字符排序

	vector <Mat> char_mat;
	for (int i = 0; i < char_rects.size(); i++)
	{
		Rect roi = char_rects[i].boundingRect();
		Mat m = inputImg;
		if (!(0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= m.cols &&
              0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= m.rows)){
			return false;
		}
		char_mat.push_back(Mat(inputImg, char_rects[i].boundingRect()));
	}
	Mat train_mat(2,3,CV_32F);//CV_32FC1
	int length;
	dst_mat.resize(7);
	Point2f srcTri[3];  
	Point2f dstTri[3];
	for (int i = 0; i < char_mat.size(); ++i)
	{
		srcTri[0] = Point2f( 0,0 );  
		srcTri[1] = Point2f( char_mat[i].cols - 1, 0 );  
		srcTri[2] = Point2f( 0, char_mat[i].rows - 1 );
		length = char_mat[i].rows > char_mat[i].cols?char_mat[i].rows:char_mat[i].cols;
		dstTri[0] = Point2f( 0.0, 0.0 );  
		dstTri[1] = Point2f( length, 0.0 );  
		dstTri[2] = Point2f( 0.0, length ); 
		train_mat = getAffineTransform( srcTri, dstTri );
		dst_mat[i] = Mat::zeros(length,length,char_mat[i].type());		
		warpAffine(char_mat[i],dst_mat[i],train_mat,dst_mat[i].size(),INTER_LINEAR,BORDER_CONSTANT,Scalar(0));
		resize(dst_mat[i],dst_mat[i],Size(20,20));//每个字符归一化为20*20的字符
	}
	return true;
}

Mat ProjectedHistogram(Mat img, int t)  
{
    int sz=(t)?img.rows:img.cols;
    Mat mhist=Mat::zeros(1,sz,CV_32F);
 
    for(int j=0; j<sz; j++){
        Mat data=(t)?img.row(j):img.col(j);
        mhist.at<float>(j)=countNonZero(data);
    }
    double min, max;
    minMaxLoc(mhist, &min, &max);
    
    if(max>0)
        mhist.convertTo(mhist,-1 , 1.0f/max, 0);
 
    return mhist;
}

void features(const Mat & in , Mat & out ,int sizeData)
{
	//分别在水平方向和垂直方向上 创建累积直方图
	Mat vhist = ProjectedHistogram(in , 1);//水平直方图
	Mat hhist = ProjectedHistogram(in , 0);//垂直直方图

	//低分辨率图像
	//低分辨率图像中的每一个像素都将被保存在特征矩阵中
	Mat lowData;
	resize(in , lowData ,Size(sizeData ,sizeData ));

	//特征矩阵的列数
	int numCols = vhist.cols + hhist.cols + lowData.cols * lowData.cols;
	out = Mat::zeros(1, numCols , CV_32F);

	//向特征矩阵赋值
	int j = 0;
	for (int i =0 ;i<vhist.cols ; ++i)//首先把水平方向累积直方图的值，存到特征矩阵中
	{
		out.at<float>(j) = vhist.at<float>(i);
		j++;
	}
	for (int i=0 ; i < hhist.cols ;++i)//然后把竖直方向累积直方图的值，存到特征矩阵中
	{
		out.at<float>(j) = hhist.at<float>(i);
	}
	for(int x =0 ;x<lowData.rows ;++x)//最后把低分辨率图像的像素值，存到特征矩阵中
	{
		for (int y =0 ;y < lowData.cols ;++ y)
		{
			out.at<float>(j) = (float)lowData.at<unsigned char>(x,y);
			j++;
		}
	}
}
//参考了http://blog.csdn.net/yiqiudream/article/details/51712497
void ann_train(CvANN_MLP &ann ,int numCharacters, int nlayers, string str)
{
	Mat trainData ,classes;
	FileStorage fs;
	fs.open(str, FileStorage::READ);

	fs["TrainingData"] >>trainData;
	fs["classes"] >>classes;

	Mat layerSizes(1,3,CV_32SC1);
	layerSizes.at<int>( 0 ) = trainData.cols;
	layerSizes.at<int>( 1 ) = nlayers; //隐藏神经元数，可设为3
	layerSizes.at<int>( 2 ) = numCharacters;//样本类数为34
	ann.create(layerSizes , CvANN_MLP::SIGMOID_SYM );//初始化ann

	Mat trainClasses;
	trainClasses.create(trainData.rows , numCharacters ,CV_32FC1);
	for (int i =0;i< trainData.rows; i++)
	{
		for (int k=0 ; k< trainClasses.cols ; k++ )
		{
			if ( k == (int)classes.at<uchar> (i))
			{
				trainClasses.at<float>(i,k)  = 1 ;
			}
			else
				trainClasses.at<float>(i,k)  = 0;			
		}		
	}

	Mat weights(1 , trainData.rows , CV_32FC1 ,Scalar::all(1) );
	ann.train( trainData ,trainClasses , weights);
}

void svm_train(CvSVM & svmClassifier)
{
	FileStorage fs;

	fs.open("SVM.xml" , FileStorage::READ);
	Mat SVM_TrainningData;
	Mat SVM_Classes;	

	fs["TrainingData"] >>SVM_TrainningData;
	fs["classes"] >>SVM_Classes;
	CvSVMParams SVM_params;
	SVM_params.kernel_type = CvSVM::LINEAR;

	svmClassifier.train(SVM_TrainningData,SVM_Classes ,Mat(),Mat(),SVM_params);//SVM训练模型

	fs.release();
}

vector<int> plr(Mat img_input)
{
	vector<int> pl;
	/******车牌定位——颜色定位******/
    Mat hsvImg ;
    cvtColor(img_input,hsvImg,CV_BGR2HSV);//将RGB模型转换成HSV模型
    vector <Mat> hsvSplit;
    split(hsvImg,hsvSplit);//分离通道
    equalizeHist(hsvSplit[2],hsvSplit[2]);//直方图均衡化,提高图像的质量
    merge(hsvSplit,hsvImg);//将分离的多个单通道合成一幅多通道图像

    const int min_blue =100;//最小蓝色区域
    const int max_blue =240;//最大蓝色区域
    int avg_h = (min_blue+max_blue)/2;
    int channels = hsvImg.channels();
    int nRows = hsvImg.rows;
    int nCols = hsvImg.cols * channels;//图像数据列需要考虑通道数的影响；

    if (hsvImg.isContinuous())//连续存储的数据，按一行处理
    {
        nCols *= nRows;
        nRows = 1;
    }

    unsigned char* p;
    const float  minref_sv = 64;//参考的SV的值
    const float max_sv = 255;//SV的最大值

    for (int i = 0; i < nRows; ++i)//根据蓝色在HSV在的区域每个通道的取值范围将此作为阈值，提取出图片中蓝色部分作为备选区域
    {
        p = hsvImg.ptr<uchar>(i);//有效提高了车牌和车色颜色在不相差较大的情况下的识别率
        for (int j = 0; j < nCols; j += 3)//问题:蓝色车的车牌无法识别
        {
            int H = int(p[j]);//0-180
            int S = int(p[j + 1]);//0-255
            int V = int(p[j + 2]);//0-255
            bool colorMatched = false;

            if (H > min_blue && H < max_blue)
            {
                int Hdiff = 0;
                float Hdiff_p = float(Hdiff) / 40;
                float min_sv = 0;

                if (H > avg_h)
                {
                    Hdiff = H - avg_h;
                }   
                else
                {
                    Hdiff = avg_h - H;
                }
                min_sv = minref_sv - minref_sv / 2 * (1 - Hdiff_p);
                if ((S > 70&& S < 255) &&(V > 70 && V < 255))
                    colorMatched = true;
            }
            if (colorMatched == true) 
            {
                p[j] = 0; p[j + 1] = 0; p[j + 2] = 255;
            }
            else 
            {
                p[j] = 0; p[j + 1] = 0; p[j + 2] = 0;
            }
        }
    }

    Mat src_grey;
    Mat img_threshold;
    vector<Mat> hsvSplit_done;

    split(hsvImg, hsvSplit_done);
    src_grey = hsvSplit_done[2];//提取黑色分量
    vector <RotatedRect>  rects;
    Mat element = getStructuringElement(MORPH_RECT ,Size(17 ,3));//闭形态学的结构元素
    morphologyEx(src_grey ,img_threshold,CV_MOP_CLOSE,element);//闭运算，先膨胀后腐蚀，连通近邻区域（填补白色区域的间隙）
    morphologyEx(img_threshold,img_threshold,MORPH_OPEN,element);//形态学处理

    vector< vector <Point> > contours;//寻找车牌区域的轮廓
    findContours(img_threshold ,contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);//只检测外轮廓。存储所有轮廓点

    //对候选的轮廓进行进一步筛选
    vector< vector <Point> > ::iterator itc = contours.begin();
    while( itc != contours.end())
    {
        RotatedRect mr = minAreaRect(Mat( *itc ));//返回每个轮廓的最小有界矩形区域
        if(!verifySizes_closeImg(mr))//判断矩形轮廓是否符合要求
        {
            itc = contours.erase(itc);
        }
        else     
        {

            rects.push_back(mr);
            ++itc;
        }      
    }
    vector <Mat> output_area;
    normal_area(img_input, rects, output_area);//获得144*33的疑似候选车牌区域

	/******采用SVM判断定位区域******/
	CvSVM  svmClassifier;
	svm_train(svmClassifier);//使用SVM对正负样本进行训练
	vector<Mat> plates_svm;

	for(int i=0;i < output_area.size(); i++)
	{
		Mat img = output_area[i];
		Mat p = img.reshape(1,1);
		p.convertTo(p,CV_32FC1);
		int response = (int)svmClassifier.predict(p);
		if (response == 1)
			plates_svm.push_back(output_area[i]);
	}
	cout << "candidate_area: " << plates_svm.size() << endl;//输出候选区域的数量

	if(plates_svm.size() != 1)  
	{
		pl.push_back(0);
		pl.push_back(0);
		return pl;//如果没有得到候选区域，则图中没有车牌
	}
	//去除车牌铆钉，顺便再进一步检测下是否为车牌区域
	if (!is_plate(plates_svm[0])){
		pl.push_back(0);
		pl.push_back(0);
		return pl;
	}

	/******车牌字符分割******/
	vector <Mat> char_seg;
	if(!char_segment(plates_svm[0],char_seg))//对车牌区域进行字符分割
	{
		pl.push_back(0);
		pl.push_back(0);
		return pl;
	}

	/******车牌字符识别******/
	vector <Mat> char_feature;
	char_feature.resize(7);
	for (int i =0; i < char_seg.size(); ++i)
		features(char_seg[i], char_feature[i], 5);
	
	//神经网络训练
	CvANN_MLP ann_classify1;//对第一个汉字进行分类建模
	ann_train(ann_classify1,3,20,"ann_xml_character.xml");
	CvANN_MLP ann_classify;//对字母和数字
	ann_train(ann_classify,34,48,"ann_xml.xml");//输入层经元数(离线训练数据集的行数)，隐藏层的神经元数，文件名字

	//字符预测
	vector<int>  char_result;
	char_result.resize(char_feature.size());
	for (int i=0;i < char_feature.size(); ++i)
	{
		if (i==0)//对汉字,汉字的识别率较低
	  	{
			Mat output(1 ,34, CV_32FC1); //1*34矩阵    
			ann_classify1.predict(char_feature[i],output);//对每个字符运用ANN.predict函数得出1*类别数的数据组（数据组中是记录这个字符跟每个类别的“相似度”）
			Point maxLoc;
			double maxVal;
			minMaxLoc(output , 0 ,&maxVal , 0 ,&maxLoc);//找出最大概率的类别
			char_result[i] =  maxLoc.x;
		}
		else//对字母和数字
		{
			Mat output(1 ,34, CV_32FC1); //1*34矩阵
			ann_classify.predict(char_feature[i],output);//预测
			Point maxLoc;
			double maxVal;
			minMaxLoc(output , 0 ,&maxVal , 0 ,&maxLoc);
			char_result[i] =  maxLoc.x;
		}
	}

	/******返回识别数据******/
	int s[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,
		//        0    1    2    3    4    5    6    7    8    9    a    b
		0x0c,0x0d,0x0e,0x0f,0x10,0x11,0x13,0x14,0x15,0x16,0x17,0x19,0x1a,
		//c     d    e    f    g    h    j    k    l    m    n    p    q
		0x1b,0x1c,0x1d,0x1e,0x1f,0x20,0x21,0x22,0x23};
		//r     s    t    u    v    w    x    y    z
	char s_check[] = {'0','1','2','3','4','5','6','7','8','9','A','B',
		'C','D','E','F','G','H','J','K','L','M','N','P','Q',
		'R','S','T','U','V','W','X','Y','Z'};
	int chinese = 0xa0;//暂时仅支持中文字符-沪的识别
	string chinese_check = "沪";
	int space = 0xff;//地域后的空格字符
	char space_check = ' ';
	int pl_l, pl_r;
	for (int w = 0; w < 3; w++)
	{     
		if (w == 0){
			pl_l = chinese;
			cout << chinese_check;
		}
		else if(w == 1){
			pl_l = (pl_l << 8) | (s[char_result[w]]);
			cout << s_check[char_result[w]];
			pl_l = (pl_l << 8) | space;
			cout << space_check;
		}
		else{
			pl_l = (pl_l << 8) | (s[char_result[w]]);
			cout << s_check[char_result[w]];
		}
	}
	pl.push_back(pl_l);

	for (int w = 3; w < char_result.size(); w++)
	{
		if(w == 3){
			pl_r = s[char_result[w]];
			cout << s_check[char_result[w]];
		}
		else{
			pl_r = (pl_r << 8) | (s[char_result[w]]);
			cout << s_check[char_result[w]];
		}
	}
	pl.push_back(pl_r);
	cout << endl;
    return pl;
}