//
//  main.cpp
//  NiUSerTracker
//
//  Created by Mikael on 08/12/2014.
//  Copyright (c) 2014 mikael. All rights reserved.
//exit(EXIT_FAILURE)

#include <iostream>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace xn;

#define NI_CHECK_ERROR(status,message) \
    if(status!=XN_STATUS_OK){ \
        cout<<message<<" failed: "<<xnGetStatusString(status)<<endl; \
        cleanup(EXIT_FAILURE); \
    }\
    else{\
        cout<<message<<" succeed !"<<endl;\
    }
#define SAMPLE_XML_PATH "./SamplesConfig.xml"

DepthGenerator depthGenerator;
ImageGenerator imageGenerator;
Context context;
ScriptNode g_scriptNode;
UserGenerator userGenerator;



/*Sortie propre*/
void cleanup(int code){
    depthGenerator.Release();
    imageGenerator.Release();
    context.Release();
    exit(code);
}

/*Callback pour un nouvel utilisateur*/
void XN_CALLBACK_TYPE newUserCallback(UserGenerator& userGenerator, XnUserID user, void *cookie){
    cout<<"Nouvel Utilisateur n°"<<endl;
    if(userGenerator.GetSkeletonCap().NeedPoseForCalibration()){
        char calibrationPose[20];
        userGenerator.GetSkeletonCap().GetCalibrationPose(calibrationPose);
        userGenerator.GetPoseDetectionCap().StartPoseDetection(calibrationPose, user);
    }
    else{
        userGenerator.GetSkeletonCap().RequestCalibration(user, true);
    }
}

/*Callback pour utilisateur perdu*/
void XN_CALLBACK_TYPE lostUserCallback(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
    cout<<"Lost user "<< nId<<endl;
}

/*Callback pour une nouvelle pose détectée*/
void XN_CALLBACK_TYPE poseDetectedCallback(PoseDetectionCapability& capability, const XnChar *poseName, XnUserID user, void *cookie){
    cout<<"Pose "<<poseName<<" detected for user "<<user<<endl;
    userGenerator.GetPoseDetectionCap().StopPoseDetection(user);
    userGenerator.GetSkeletonCap().RequestCalibration(user, true);
}

/*Callback pour une calibration accomplie*/
void XN_CALLBACK_TYPE calibrationCompleteCallback(SkeletonCapability& capability, XnUserID user, XnCalibrationStatus status, void *cookie){
    if(status==XN_CALIBRATION_STATUS_OK){
        cout<<"Calibration complete, start tracking user "<<user<<endl;
        userGenerator.GetSkeletonCap().StartTracking(user);
    }
    else if(userGenerator.GetSkeletonCap().NeedPoseForCalibration()){
        char calibrationPose[20];
        userGenerator.GetSkeletonCap().GetCalibrationPose(calibrationPose);
        userGenerator.GetPoseDetectionCap().StartPoseDetection(calibrationPose, user);
    }
    else{
        userGenerator.GetSkeletonCap().RequestCalibration(user, true);
    }
}

/*Tracé des joints trackés*/
void drawJoint(DepthGenerator depthGenerator, UserGenerator userGenerator, IplImage *img, XnUserID user, XnSkeletonJoint joint){
    if(!userGenerator.GetSkeletonCap().IsTracking(user)) return;
    XnSkeletonJointPosition jointPosition;
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, jointPosition);
    if(jointPosition.fConfidence==0){
        return;
    }
    XnPoint3D projectivePosition;
    depthGenerator.ConvertRealWorldToProjective(1, &jointPosition.position, &projectivePosition);
    CvPoint p = cvPoint(projectivePosition.X, projectivePosition.Y);
    cvCircle(img, p, 10, cvScalar(255,0,0),3,8,0);
}

/*Affichage des images de profondeur*/
void displayRangeImage(const char *name, const IplImage *image, const uint16_t maxDepth){
    IplImage *normalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    for(int row=0; row<image->height;row++){
        for(int col=0;col<image->width;col++){
            CV_IMAGE_ELEM(normalized, uint8_t, row, col)=(255* CV_IMAGE_ELEM(image, uint16_t, row, col))/maxDepth;
        }
    }
    cvShowImage(name,normalized);
    cvReleaseImage(&normalized);
}

/*Affichage des images couleur*/
void displayRGBImage(const char *name,const IplImage *image){
    IplImage *imageBGR = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
    cvCvtColor(image, imageBGR, CV_RGB2BGR);
    cvShowImage(name, imageBGR);
    cvReleaseImage(&imageBGR);
}



int main(int argc, const char * argv[]) {
    
    cout << "Hello, World!\n";
    XnStatus status;
    EnumerationErrors errors;
    /*Initialisation du context OpenNi (indispensable)*/
    status = context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
    NI_CHECK_ERROR(status,"Initialisation d'OpenNi");
    
    /*Création des noeuds de production*/
    status=context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
    NI_CHECK_ERROR(status, "Création du noeud de production de la profondeur de champ");
    status=context.FindExistingNode(XN_NODE_TYPE_IMAGE, imageGenerator);
    NI_CHECK_ERROR(status, "Création du noeud de production des images RGB");

    /*Recadrage du noeud de profondeur / image couleur*/
    status = depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
    NI_CHECK_ERROR(status, "Recadrage du noeud de profondeur");
    
    /*Récupération des données et affichage*/
    /*Récupération des metadata pour créer les interfaces opencv*/
    ImageMetaData imageMD;
    DepthMetaData depthMD;
    depthGenerator.GetMetaData(depthMD);
    imageGenerator.GetMetaData(imageMD);
    IplImage *depthMap = cvCreateImage(cvSize(depthMD.FullXRes(),
                                              depthMD.FullYRes()),
                                       IPL_DEPTH_16U, 1);
    IplImage *imageMap = cvCreateImage(cvSize(imageMD.FullXRes(),
                                              imageMD.FullYRes()),
                                       IPL_DEPTH_8U, 3);
    uint16_t maxDepth = depthMD.ZRes();
    depthMD.Free();
    imageMD.Free();
    
    /*Générateur d'utilisateur*/
    status = userGenerator.Create(context);
    NI_CHECK_ERROR(status, "Creation du user generator");
    
    /*Création des callbacks*/
    XnCallbackHandle hNewUserCallbacks, hPoseDetectedCallbacks, hCalibrationCompleteCallbacks;
    
    /*Enregistrement des callbacks*/
    status = userGenerator.RegisterUserCallbacks(newUserCallback, lostUserCallback,NULL,  hNewUserCallbacks);
    NI_CHECK_ERROR(status, "Register to user callbacks");
    
    status = userGenerator.GetPoseDetectionCap().RegisterToPoseDetected(poseDetectedCallback, NULL, hPoseDetectedCallbacks);
    NI_CHECK_ERROR(status, "Register to pose detected callbacks");
    
    status = userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(calibrationCompleteCallback, NULL, hCalibrationCompleteCallbacks);
    NI_CHECK_ERROR(status, "Register to calibration complete callbacks");
    
    /*Définition des articulations suivies*/
    userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);
    
    /*Initialisation de la fenêtre*/
    status = context.StartGeneratingAll();
    NI_CHECK_ERROR(status, "Génération des noeuds de production");
    
    
    /*Boucle principale*/
    bool die=false;
    while (!die) {
        status=context.WaitOneUpdateAll(depthGenerator); //WaitAndUptdateAll()
        if(status!=XN_STATUS_OK){
            cout<<"Updating context failed: "<<xnGetStatusString(status)<<endl;
            cleanup(EXIT_FAILURE);
        }
        
        /*Copie des données openni dans les structures opencv*/
        memcpy(depthMap->imageData, depthGenerator.GetDepthMap(), depthMap->imageSize);
        memcpy(imageMap->imageData, imageGenerator.GetRGB24ImageMap(), imageMap->imageSize);
        
        /*Calcul de la distance moyenne entre Kinect et chacun des pixels*/
        /*float avgDist = 0;
        int validPixels = 0;
        XnPoint3D *projectiveCoordinates = (XnPoint3D*) malloc(sizeof(XnPoint3D) * depthMap->width * depthMap->height);
        XnPoint3D *realCoordinates = (XnPoint3D*) malloc(sizeof(XnPoint3D) * depthMap->width * depthMap->height);
        for(int row=0; row<depthMap->height;row++){
            for(int col=0; col<depthMap->width;col++){
                uint16_t depth = CV_IMAGE_ELEM(depthMap, uint16_t, row, col);
                if(depth==0) continue;
                XnPoint3D *p = &projectiveCoordinates[validPixels++];
                p->X = row; p->Y = col; p->Z = depth;
            }
        }
        depthGenerator.ConvertProjectiveToRealWorld(validPixels, projectiveCoordinates, realCoordinates);
        for( int i =0; i<validPixels; i++){
            XnPoint3D p = realCoordinates[i];
            avgDist += sqrt(p.X*p.X + p.Y*p.Y + p.Z*p.Z);
        }
        avgDist /=validPixels;*/
        
        /*Ensemble des utilisateurs détectés*/
        uint16_t userCount = userGenerator.GetNumberOfUsers();
        XnUserID users[userCount];
        userGenerator.GetUsers(users, userCount);
        
        /*On vérifie si le squelette de chq user est suivi*/
        for(int i =0; i<userCount;i++){
            if(userGenerator.GetSkeletonCap().IsTracking(users[i])){
                drawJoint(depthGenerator,userGenerator,imageMap,users[i],XN_SKEL_LEFT_HAND);
                drawJoint(depthGenerator,userGenerator,imageMap,users[i],XN_SKEL_RIGHT_HAND);
                drawJoint(depthGenerator,userGenerator,imageMap,users[i],XN_SKEL_HEAD);
                drawJoint(depthGenerator,userGenerator,imageMap,users[i],XN_SKEL_TORSO);
            }
        }
        displayRangeImage("Image de profondeur", depthMap, maxDepth);
        displayRGBImage("Image couleur", imageMap);
        int c = cvWaitKey (2); //attente de 2ms qu'une touche soit pressée, !! permet le rafraîchissement des images !!
        if (c==113) //valeur de 'q'
        { die = true;}
        
    }
    
    /*Libération du context et des générateurs*/
    cleanup(0);
    }
