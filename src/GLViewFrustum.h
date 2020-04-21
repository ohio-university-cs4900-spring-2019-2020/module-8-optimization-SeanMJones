#pragma once

#include "GLView.h";
#include <WOFTGLString.h>

namespace Aftr
{
    class Camera;

    class GLViewFrustum : public GLView {
        public:
           static GLViewFrustum* New( const std::vector< std::string >& outArgs );
           virtual ~GLViewFrustum();
           virtual void init(float gScalar, Vector gravityNormVec, std::string confFileName, const PHYSICS_ENGINE_TYPE& physEType);
           virtual void updateWorld(); ///< Called once per frame
           virtual void loadMap(); ///< Called once at startup to build this module's scene
           virtual void onResizeWindow( GLsizei width, GLsizei height );
           virtual void onMouseDown( const SDL_MouseButtonEvent& e );
           virtual void onMouseUp( const SDL_MouseButtonEvent& e );
           virtual void onMouseMove( const SDL_MouseMotionEvent& e );
           virtual void onKeyDown( const SDL_KeyboardEvent& key );
           virtual void onKeyUp( const SDL_KeyboardEvent& key );
           virtual void spawnBoxes(int boxCount = 500);
        
        protected:
           GLViewFrustum( const std::vector< std::string >& args );
           virtual void onCreate();
           bool inFrustum(WO* wo, const AftrGeometryFrustum& frustum) const;
           bool frustumVisible(WO* wo) const;
        
           float nearPlaneDist = 10.0f;
           float farPlaneDist = 50.0f;
           float aspectRatio = 3.0f;
           float horizontalFOV = 50.0f;
           float verticalFOV = 2 * atan(tan(horizontalFOV * Aftr::DEGtoRAD * .5f) / aspectRatio) * Aftr::RADtoDEG;
           std::vector<WO*> boxes;
           WO* frustum;
           WO* frustumCam;
};

} 
