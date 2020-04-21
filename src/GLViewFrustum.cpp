#include "GLViewFrustum.h"

#include "WorldList.h" //This is where we place all of our WOs
#include "ManagerOpenGLState.h" //We can change OpenGL State attributes with this
#include "Axes.h" //We can set Axes to on/off with this
#include "PhysicsEngineODE.h"

//Different WO used by this module
#include "WO.h"
#include "WOStatic.h"
#include "WOStaticPlane.h"
#include "WOStaticTrimesh.h"
#include "WOTrimesh.h"
#include "WOHumanCyborg.h"
#include "WOHumanCal3DPaladin.h"
#include "WOWayPointSpherical.h"
#include "WOLight.h"
#include "WOSkyBox.h"
#include "WOCar1970sBeater.h"
#include "Camera.h"
#include "CameraStandard.h"
#include "CameraChaseActorSmooth.h"
#include "CameraChaseActorAbsNormal.h"
#include "CameraChaseActorRelNormal.h"
#include "Model.h"
#include "ModelDataShared.h"
#include "ModelMesh.h"
#include "ModelMeshDataShared.h"
#include "ModelMeshSkin.h"
#include "WONVStaticPlane.h"
#include "WONVPhysX.h"
#include "WONVDynSphere.h"
#include "AftrGLRendererBase.h"
#include "MGLFrustum.h"



//If we want to use way points, we need to include this.
#include "io.h";
#include <WOFTGLString.h>
#include <MGLFTGLString.h>


using namespace Aftr;

GLViewFrustum* GLViewFrustum::New( const std::vector< std::string >& args )
{
   GLViewFrustum* glv = new GLViewFrustum( args );
   glv->init( Aftr::GRAVITY, Vector( 0, 0, -1.0f ), "aftr.conf", PHYSICS_ENGINE_TYPE::petODE );
   glv->onCreate();
   return glv;
}

void GLViewFrustum::init(float gScalar, Vector gNormVec, std::string confFileName, const PHYSICS_ENGINE_TYPE& physEType) {
    GLView::init(gScalar, gNormVec, confFileName, physEType);
}


GLViewFrustum::GLViewFrustum( const std::vector< std::string >& args ) : GLView( args )
{
   //Initialize any member variables that need to be used inside of LoadMap() here.
   //Note: At this point, the Managers are not yet initialized. The Engine initialization
   //occurs immediately after this method returns (see GLViewFrustum::New() for
   //reference). Then the engine invoke's GLView::loadMap() for this module.
   //After loadMap() returns, GLView::onCreate is finally invoked.

   //The order of execution of a module startup:
   //GLView::New() is invoked:
   //    calls GLView::init()
   //       calls GLView::loadMap() (as well as initializing the engine's Managers)
   //    calls GLView::onCreate()

   //GLViewFrustum::onCreate() is invoked after this module's LoadMap() is completed.
}

void GLViewFrustum::onKeyDown(const SDL_KeyboardEvent& key)
{
    GLView::onKeyDown(key);
    if (key.keysym.sym == SDLK_0)
        this->setNumPhysicsStepsPerRender(1);
    if (key.keysym.sym == SDLK_UP) {
        this->cam->moveInLookDirection(2);
    }
    if (key.keysym.sym == SDLK_DOWN) {
        this->cam->moveOppositeLookDirection(2);
    }
    if (key.keysym.sym == SDLK_LEFT) {
        this->cam->moveLeft();
        this->cam->moveLeft();
        this->cam->moveLeft();
        this->cam->moveLeft();
        this->cam->moveLeft();
    }
    if (key.keysym.sym == SDLK_RIGHT) {
        this->cam->moveRight();
        this->cam->moveRight();
        this->cam->moveRight();
        this->cam->moveRight();
        this->cam->moveRight();
    }
    if (key.keysym.sym == SDLK_q) {
        Vector newPos = this->frustum->getPosition();
        newPos.z = newPos.z += 5;
        this->frustum->setPosition(newPos);

    }
    if (key.keysym.sym == SDLK_e) {
        Vector newPos = this->frustum->getPosition();
        newPos.z = newPos.z -= 5;
        this->frustum->setPosition(newPos);
    }
}


void GLViewFrustum::onCreate()
{
    //GLViewFrustum::onCreate() is invoked after this module's LoadMap() is completed.
    //At this point, all the managers are initialized. That is, the engine is fully initialized.

    if (this->pe != NULL)
    {

        this->pe->setGravityNormalizedVector(Vector(0, 0, -1.0f));
        this->pe->setGravityScalar(Aftr::GRAVITY);
    }
    this->setActorChaseType(STANDARDEZNAV);
}

void GLViewFrustum::updateWorld()
{
    GLView::updateWorld(); //Just call the parent's update world first.
                           //If you want to add additional functionality, do it after
                           //this call.

    // calculate delta time
    static auto last_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_time).count();
    last_time = now;

    Mat4 cdm = this->cam->getDisplayMatrix();
    Vector pos = this->cam->getPosition();
    frustumCam->getModel()->setDisplayMatrix(cdm);
    frustumCam->setPosition(pos);

    Mat4 fdm = frustum->getDisplayMatrix();
    fdm = fdm.rotate(Vector(0.0f, 0.0f, 1.0f), dt * Aftr::PI / 5.0f);
    frustum->getModel()->setDisplayMatrix(fdm);

    for (int i = 0; i < boxes.size(); ++i) {

        if (frustumVisible(boxes[i])) {
            boxes[i]->isVisible = true;
        }
        else {
            boxes[i]->isVisible = false;
        }
    }

}


void Aftr::GLViewFrustum::loadMap()
{
   this->worldLst = new WorldList(); //WorldList is a 'smart' vector that is used to store WO*'s
   this->actorLst = new WorldList();
   this->netLst = new WorldList();

   ManagerOpenGLState::GL_CLIPPING_PLANE = 1000.0;
   ManagerOpenGLState::GL_NEAR_PLANE = 0.1f;
   ManagerOpenGLState::enableFrustumCulling = false;
   Axes::isVisible = true;
   this->glRenderer->isUsingShadowMapping( false ); //set to TRUE to enable shadow mapping, must be using GL 3.2+

   this->cam->setPosition(-10,0,15);

   std::string shinyRedPlasticCube( ManagerEnvironmentConfiguration::getSMM() + "/models/cube4x4x4redShinyPlastic_pp.wrl" );
   std::string wheeledCar( ManagerEnvironmentConfiguration::getSMM() + "/models/rcx_treads.wrl" );
   std::string grass( ManagerEnvironmentConfiguration::getSMM() + "/models/grassFloor400x400_pp.wrl" );
   std::string human( ManagerEnvironmentConfiguration::getSMM() + "/models/human_chest.wrl" );

   /*player = WO::New(ManagerEnvironmentConfiguration::getSMM() + "models/box/box.blend", Vector(1, 1, 1), MESH_SHADING_TYPE::mstFLAT);
   player->setPosition(Vector(0, 0, 10));
   worldLst->push_back(player);
   std::string times = ManagerEnvironmentConfiguration::getSMM() + "fonts/TIMES.TTF";
   label = WOFTGLString::New(times, 30);
   usermsg = "Player1";
   label->setText(usermsg);
   label->setPosition(0, 0, 12);
   label->getModelT<MGLFTGLString>()->setSize(2, 2);
   label->getModelT<MGLFTGLString>()->rotateAboutGlobalX(1.55);
   label->getModelT<MGLFTGLString>()->rotateAboutGlobalZ(-1.5);
   worldLst->push_back(label);*/

   //SkyBox Textures readily available
   std::vector< std::string > skyBoxImageNames; //vector to store texture paths
   skyBoxImageNames.push_back(ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_evening+6.jpg");

   float ga = 0.1f; //Global Ambient Light level for this module
   ManagerLight::setGlobalAmbientLight( aftrColor4f( ga, ga, ga, 1.0f ) );
   WOLight* light = WOLight::New();
   light->isDirectionalLight( true );
   light->setPosition( Vector( 0, 0, 100 ) );
   //Set the light's display matrix such that it casts light in a direction parallel to the -z axis (ie, downwards as though it was "high noon")
   //for shadow mapping to work, this->glRenderer->isUsingShadowMapping( true ), must be invoked.
   light->getModel()->setDisplayMatrix( Mat4::rotateIdentityMat( { 0, 1, 0 }, 90.0f * Aftr::DEGtoRAD ) );
   light->setLabel( "Light" );
   worldLst->push_back( light );

   //Create the SkyBox
   WO* wo = WOSkyBox::New( skyBoxImageNames.at( 0 ), this->getCameraPtrPtr() );
   wo->setPosition( Vector( 0,0,0 ) );
   wo->setLabel( "Sky Box" );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   worldLst->push_back( wo );

   ////Create the infinite grass plane (the floor)
   wo = WO::New( grass, Vector( 1, 1, 1 ), MESH_SHADING_TYPE::mstFLAT );
   wo->setPosition( Vector( 0, 0, 0 ) );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   ModelMeshSkin& grassSkin = wo->getModel()->getModelDataShared()->getModelMeshes().at( 0 )->getSkins().at( 0 );
   grassSkin.getMultiTextureSet().at( 0 )->setTextureRepeats( 5.0f );
   grassSkin.setAmbient( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Color of object when it is not in any light
   grassSkin.setDiffuse( aftrColor4f( 1.0f, 1.0f, 1.0f, 1.0f ) ); //Diffuse color components (ie, matte shading color of this object)
   grassSkin.setSpecular( aftrColor4f( 0.4f, 0.4f, 0.4f, 1.0f ) ); //Specular color component (ie, how "shiney" it is)
   grassSkin.setSpecularCoefficient( 10 ); // How "sharp" are the specular highlights (bigger is sharper, 1000 is very sharp, 10 is very dull)
   wo->setLabel( "Grass" );
   worldLst->push_back( wo );

   ////Create the infinite grass plane that uses the Open Dynamics Engine (ODE)
   //wo = WOStatic::New( grass, Vector(1,1,1), MESH_SHADING_TYPE::mstFLAT );
   //((WOStatic*)wo)->setODEPrimType( ODE_PRIM_TYPE::PLANE );
   //wo->setPosition( Vector(0,0,0) );
   //wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0).getMultiTextureSet().at(0)->setTextureRepeats( 5.0f );
   //wo->setLabel( "Grass" );
   //worldLst->push_back( wo );

   //Create the infinite grass plane that uses NVIDIAPhysX(the floor)
   /*wo = WONVStaticPlane::New( grass, Vector(1,1,1), MESH_SHADING_TYPE::mstFLAT );
   wo->setPosition( Vector(0,0,0) );
   wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   wo->getModel()->getModelDataShared()->getModelMeshes().at(0)->getSkins().at(0).getMultiTextureSet().at(0)->setTextureRepeats( 5.0f );
   wo->setLabel( "Grass" );
   worldLst->push_back( wo );*/

   ////Create the infinite grass plane (the floor)
   //wo = WONVPhysX::New( shinyRedPlasticCube, Vector(1,1,1), MESH_SHADING_TYPE::mstFLAT );
   //wo->setPosition( Vector(0,0,50.0f) );
   //wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //wo->setLabel( "Grass" );
   //worldLst->push_back( wo );

   //wo = WONVPhysX::New( shinyRedPlasticCube, Vector(1,1,1), MESH_SHADING_TYPE::mstFLAT );
   //wo->setPosition( Vector(0,0.5f,75.0f) );
   //wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
   //wo->setLabel( "Grass" );
   //worldLst->push_back( wo );

   //wo = WONVDynSphere::New( ManagerEnvironmentConfiguration::getVariableValue("sharedmultimediapath") + "/models/sphereRp5.wrl", Vector(1.0f, 1.0f, 1.0f), mstSMOOTH );
   //wo->setPosition( 0,0,100.0f );
   //wo->setLabel( "Sphere" );
   //this->worldLst->push_back( wo );

   //wo = WOHumanCal3DPaladin::New( Vector( .5, 1, 1 ), 100 );
   //((WOHumanCal3DPaladin*)wo)->rayIsDrawn = false; //hide the "leg ray"
   //((WOHumanCal3DPaladin*)wo)->isVisible = false; //hide the Bounding Shell
   //wo->setPosition( Vector(20,20,20) );
   //wo->setLabel( "Paladin" );
   //worldLst->push_back( wo );
   //actorLst->push_back( wo );
   //netLst->push_back( wo );
   //this->setActor( wo );
   //
   //wo = WOHumanCyborg::New( Vector( .5, 1.25, 1 ), 100 );
   //wo->setPosition( Vector(20,10,20) );
   //wo->isVisible = false; //hide the WOHuman's bounding box
   //((WOHuman*)wo)->rayIsDrawn = false; //show the 'leg' ray
   //wo->setLabel( "Human Cyborg" );
   //worldLst->push_back( wo );
   //actorLst->push_back( wo ); //Push the WOHuman as an actor
   //netLst->push_back( wo );
   //this->setActor( wo ); //Start module where human is the actor

   ////Create and insert the WOWheeledVehicle
   //std::vector< std::string > wheels;
   //std::string wheelStr( "../../../shared/mm/models/WOCar1970sBeaterTire.wrl" );
   //wheels.push_back( wheelStr );
   //wheels.push_back( wheelStr );
   //wheels.push_back( wheelStr );
   //wheels.push_back( wheelStr );
   //wo = WOCar1970sBeater::New( "../../../shared/mm/models/WOCar1970sBeater.wrl", wheels );
   //wo->setPosition( Vector( 5, -15, 20 ) );
   //wo->setLabel( "Car 1970s Beater" );
   //((WOODE*)wo)->mass = 200;
   //worldLst->push_back( wo );
   //actorLst->push_back( wo );
   //this->setActor( wo );
   //netLst->push_back( wo );

   spawnBoxes();

   frustum = WO::New();
   frustum->setModel(MGLFrustum::New(frustum, nearPlaneDist, farPlaneDist, horizontalFOV, aspectRatio));
   frustum->setPosition(0, 0, 15.0f);
   worldLst->push_back(frustum);



   frustumCam = WO::New();
   frustumCam->setModel(MGLFrustum::New(frustumCam, nearPlaneDist, farPlaneDist, horizontalFOV, aspectRatio));
   Mat4 camOrient = this->cam->getDisplayMatrix();
   frustumCam->getModel()->setDisplayMatrix(camOrient);
   Vector camPos = this->cam->getPosition();
   frustumCam->setPosition(camPos);
   worldLst->push_back(frustumCam);
}

void GLViewFrustum::spawnBoxes(int boxCount) {
    for (int i = 0; i <= boxCount; ++i) {
        WO* box = WO::New(ManagerEnvironmentConfiguration::getSMM() + "models/box/box.blend", Vector(1.0f, 1.0f, 1.0f), MESH_SHADING_TYPE::mstFLAT);
        box->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
        box->setPosition(rand() % 100 - 50, rand() % 100 - 50, rand() % 100);
        box->getModel()->renderBBox = false;
        worldLst->push_back(box);
        boxes.push_back(box);
    }
}

bool GLViewFrustum::inFrustum(WO* wo, const AftrGeometryFrustum& frustum) const {
    Mat4 mat = wo->getDisplayMatrix();
    Vector pos = wo->getPosition();
    Vector boxHalf = wo->getModel()->getBoundingBox().getlxlylz() * 0.5f;
    for (unsigned int plane = 0; plane < 6; ++plane) {
        Vector normal = frustum.getPlaneNormal(plane);
        float coeff = frustum.getPlaneCoef(plane);

        // check if each vertex of the bbox is outside this frustum plane
        bool outFrustum = true;
        for (int x = -1; x <= 1; x += 2) {
            for (int y = -1; y <= 1; y += 2) {
                for (int z = -1; z <= 1; z += 2) {
                    Vector v = mat * (Vector(x, y, z) * boxHalf) + pos;
                    if (normal.dotProduct(v) < coeff) {
                        outFrustum = false;
                    }
                }
            }
        }

        if (outFrustum) {
            return false;
        }
    }

    return true;
}

bool GLViewFrustum::frustumVisible(WO* wo) const {
    Vector look = frustum->getLookDirection();
    Vector norm = frustum->getNormalDirection();
    Vector pos = frustum->getPosition();
    Vector lookCam = frustumCam->getLookDirection();
    Vector normCam = frustumCam->getNormalDirection();
    Vector posCam = frustumCam->getPosition();

    AftrGeometryFrustum frust(aspectRatio, verticalFOV, nearPlaneDist, farPlaneDist, look, norm, pos);
    AftrGeometryFrustum frust_cam(aspectRatio, verticalFOV, nearPlaneDist, farPlaneDist, lookCam, normCam, posCam);

    return inFrustum(wo, frust) || inFrustum(wo, frust_cam);
}

GLViewFrustum::~GLViewFrustum()
{

    //Implicitly calls GLView::~GLView()
}


void GLViewFrustum::onResizeWindow(GLsizei width, GLsizei height)
{
    GLView::onResizeWindow(width, height); //call parent's resize method.
}


void GLViewFrustum::onMouseDown(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseDown(e);
}


void GLViewFrustum::onMouseUp(const SDL_MouseButtonEvent& e)
{
    GLView::onMouseUp(e);
}


void GLViewFrustum::onMouseMove(const SDL_MouseMotionEvent& e)
{
    GLView::onMouseMove(e);
}

void GLViewFrustum::onKeyUp(const SDL_KeyboardEvent& key)
{
    GLView::onKeyUp(key);
}

//Skyboxes
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_water+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_dust+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_mountains+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_winter+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/early_morning+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_afternoon+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_cloudy3+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_day2+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_deepsun+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_morning2+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_noon+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/sky_warp+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_Hubble_Nebula+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_gray_matter+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_easter+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_hot_nebula+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_ice_field+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_lemon_lime+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_milk_chocolate+6.jpg" );
//skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_solar_bloom+6.jpg" );
 //skyBoxImageNames.push_back( ManagerEnvironmentConfiguration::getSMM() + "/images/skyboxes/space_thick_rb+6.jpg" );