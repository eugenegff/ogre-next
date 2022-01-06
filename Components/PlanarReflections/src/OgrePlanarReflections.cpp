/*
-----------------------------------------------------------------------------
This source file is part of OGRE-Next
    (Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org/

Copyright (c) 2000-2017 Torus Knot Software Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
-----------------------------------------------------------------------------
*/

#include "OgrePlanarReflections.h"

#include "Compositor/OgreCompositorManager2.h"
#include "Compositor/OgreCompositorWorkspace.h"
#include "Math/Array/OgreBooleanMask.h"
#include "OgreCamera.h"
#include "OgreHlms.h"
#include "OgreHlmsDatablock.h"
#include "OgreLogManager.h"
#include "OgrePixelFormatGpuUtils.h"
#include "OgreSceneManager.h"
#include "OgreTextureGpuManager.h"

#include <math.h>
#include "OgreEntity.h"
#ifdef OGRE_BELIGHT_MINI
#include "OgreRoot.h"
#endif

namespace Ogre
{
    // clang-format off
    static const Ogre::Matrix4 PROJECTIONCLIPSPACE2DTOIMAGESPACE_PERSPECTIVE(
            0.5,    0,    0,  0.5,
            0,   -0.5,    0,  0.5,
            0,      0,    1,    0,
            0,      0,    0,    1 );
    // clang-format on

    PlanarReflections::PlanarReflections( SceneManager *sceneManager,
                                          CompositorManager2 *compositorManager, Real maxDistance,
                                          Camera *lockCamera ) :
        mActorsSoA( 0 ),
        mCapacityActorsSoA( 0 ),
        mLastAspectRatio( 0 ),
        mLastCameraPos( Vector3::ZERO ),
        mLastCameraRot( Quaternion::IDENTITY ),
        mLastCamera( 0 ),
        // mLockCamera( lockCamera ),
        mUpdatingRenderablesHlms( false ),
        mAnyPendingFlushRenderable( false ),
        mMaxNumMipmaps( 0u ),
        mMaxActiveActors( 0u ),
        mInvMaxDistance( Real( 1.0 ) / maxDistance ),
        mMaxSqDistance( maxDistance * maxDistance ),
        mSceneManager( sceneManager ),
        mCompositorManager( compositorManager ),
        mDummyActor()
    {
    }
    //-----------------------------------------------------------------------------------
    PlanarReflections::~PlanarReflections()
    {
        destroyAllActors();
        if( mActorsSoA )
        {
            OGRE_FREE_SIMD( mActorsSoA, MEMCATEGORY_SCENE_OBJECTS );
            mActorsSoA = 0;
            mCapacityActorsSoA = 0;
        }

#ifndef OGRE_BELIGHT_MINI
        TextureGpuManager *textureGpuManager =
            mSceneManager->getDestinationRenderSystem()->getTextureGpuManager();
#endif

        ActiveActorDataVec::const_iterator itor = mActiveActorData.begin();
        ActiveActorDataVec::const_iterator end = mActiveActorData.end();

        while( itor != end )
        {
            mCompositorManager->removeWorkspace( itor->workspace );
            mSceneManager->destroyCamera( itor->reflectionCamera );
#ifndef OGRE_BELIGHT_MINI
            textureGpuManager->destroyTexture( itor->reflectionTexture );
#endif
            ++itor;
        }

        mActiveActorData.clear();
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::setMaxDistance( Real maxDistance )
    {
        mMaxSqDistance = maxDistance * maxDistance;
        mInvMaxDistance = Real( 1.0 ) / maxDistance;
    }
    //-----------------------------------------------------------------------------------
#ifdef OGRE_BELIGHT_MINI
    void PlanarReflections::setMaxActiveActors( IdString workspaceName, bool useAccurateLighting, const std::vector<Ogre::TextureGpu*>& sharedExternalTextures, TextureGpu* sharedWorkMSAATexture )
    {
        uint8 maxActiveActors = sharedExternalTextures.size();
        if(mMaxActiveActors!=0 || !mActiveActorData.empty())
        {
            destroyAllActors();
            if( mActorsSoA )
            {
                OGRE_FREE_SIMD( mActorsSoA, MEMCATEGORY_SCENE_OBJECTS );
                mActorsSoA = 0;
                mCapacityActorsSoA = 0;
            }

            ActiveActorDataVec::const_iterator itor = mActiveActorData.begin();
            ActiveActorDataVec::const_iterator end  = mActiveActorData.end();

            while( itor != end )
            {
                mCompositorManager->removeWorkspace( itor->workspace );
                mSceneManager->destroyCamera( itor->reflectionCamera );
                ++itor;
            }

            mActiveActorData.clear();
            mMaxActiveActors = 0;
        }
        
        mMaxNumMipmaps = 0u;
        mMaxActiveActors = maxActiveActors;
        if( mMaxActiveActors>0 )
        {
            mActiveActorData.reserve( mMaxActiveActors );
            const size_t numNewActiveActors = mMaxActiveActors;
            for( size_t i=0; i<numNewActiveActors; ++i )
            {
                ActiveActorData actorData;
                const size_t uniqueId = Id::generateNewId<ActiveActorData>();
                String cameraName = "PlanarReflectionActor #" + StringConverter::toString( uniqueId );
                actorData.reflectionCamera = mSceneManager->createCamera( cameraName, useAccurateLighting );
                actorData.reflectionCamera->setAutoAspectRatio( false );

                actorData.reflectionTexture = sharedExternalTextures[i];
                
                mMaxNumMipmaps = std::max( actorData.reflectionTexture->getNumMipmaps(), mMaxNumMipmaps );
                
                CompositorChannelVec channels;
                channels.push_back( actorData.reflectionTexture );
                if(sharedWorkMSAATexture)
                    channels.push_back( sharedWorkMSAATexture );

                actorData.workspace = mCompositorManager->addWorkspace( mSceneManager, channels,
                                                                        actorData.reflectionCamera,
                                                                        workspaceName, false, 0 );
                actorData.isReserved = false;
                mActiveActorData.push_back( actorData );
            }
        }
    }
#else
    void PlanarReflections::setMaxActiveActors( uint8 maxActiveActors, IdString workspaceName,
                                                bool useAccurateLighting, uint32 width, uint32 height,
                                                bool withMipmaps, PixelFormatGpu pixelFormat,
                                                bool mipmapMethodCompute )
    {
        if( maxActiveActors < mMaxActiveActors )
        {
            TextureGpuManager *textureGpuManager =
                mSceneManager->getDestinationRenderSystem()->getTextureGpuManager();

            // Shrinking
            ActiveActorDataVec::const_iterator itor = mActiveActorData.begin() + maxActiveActors;
            ActiveActorDataVec::const_iterator end = mActiveActorData.end();

            while( itor != end )
            {
                mCompositorManager->removeWorkspace( itor->workspace );
                mSceneManager->destroyCamera( itor->reflectionCamera );
                textureGpuManager->destroyTexture( itor->reflectionTexture );
                if( itor->isReserved )
                {
                    const size_t slotIdx = size_t( itor - mActiveActorData.begin() );
                    PlanarReflectionActorVec::iterator itActor = mActors.begin();
                    PlanarReflectionActorVec::iterator enActor = mActors.end();

                    while( itActor != enActor && ( !( *itActor )->hasReservation() ||
                                                   ( *itActor )->mCurrentBoundSlot != slotIdx ) )
                    {
                        ++itActor;
                    }

                    assert( itActor != enActor &&
                            "Slot was reserved but we couldn't find "
                            "any Actor that had that reservation" );

                    ( *itActor )->mHasReservation = false;
                    ( *itActor )->mCurrentBoundSlot = 0xFF;
                }

                ++itor;
            }

            mActiveActorData.resize( maxActiveActors );

            // Recalculate mMaxNumMipmaps
            mMaxNumMipmaps = 0u;
            itor = mActiveActorData.begin() + maxActiveActors;
            end = mActiveActorData.end();

            while( itor != end )
            {
                mMaxNumMipmaps = std::max( itor->reflectionTexture->getNumMipmaps(), mMaxNumMipmaps );
                ++itor;
            }
        }

        const uint8 oldValue = mMaxActiveActors;
        mMaxActiveActors = maxActiveActors;

        if( oldValue < mMaxActiveActors )
        {
            // Enlarging
            mActiveActorData.reserve( mMaxActiveActors );
            const size_t numNewActiveActors = mMaxActiveActors - oldValue;
            for( size_t i = 0; i < numNewActiveActors; ++i )
            {
                ActiveActorData actorData;
                const size_t uniqueId = Id::generateNewId<PlanarReflections>();
                String cameraName = "PlanarReflectionActor #" + StringConverter::toString( uniqueId );
                actorData.reflectionCamera =
                    mSceneManager->createCamera( cameraName, useAccurateLighting );
                actorData.reflectionCamera->setAutoAspectRatio( false );

                uint32 textureFlags = TextureFlags::RenderToTexture;
                textureFlags |= ( withMipmaps && mipmapMethodCompute ) ? TextureFlags::Uav
                                                                       : TextureFlags::AllowAutomipmaps;
                uint32 numMips =
                    !withMipmaps ? 1u : PixelFormatGpuUtils::getMaxMipmapCount( width, height, 1u );
                numMips = numMips - std::min( 4u, numMips - 1u );  // Mips below 16x16 are problematic.

                // Always use HW Gamma, except when using compute mipmaps, which has issues.
                const bool hwGamma = !( withMipmaps && mipmapMethodCompute );
                if( hwGamma )
                    pixelFormat = PixelFormatGpuUtils::getEquivalentSRGB( pixelFormat );
                else
                    pixelFormat = PixelFormatGpuUtils::getEquivalentLinear( pixelFormat );

                TextureGpuManager *textureGpuManager =
                    mSceneManager->getDestinationRenderSystem()->getTextureGpuManager();
                actorData.reflectionTexture = textureGpuManager->createTexture(
                    "PlanarReflections #" + StringConverter::toString( uniqueId ),
                    GpuPageOutStrategy::Discard, textureFlags, TextureTypes::Type2D );
                actorData.reflectionTexture->setResolution( width, height );
                actorData.reflectionTexture->setPixelFormat( pixelFormat );
                actorData.reflectionTexture->setNumMipmaps( static_cast<uint8>( numMips ) );
                actorData.reflectionTexture->_transitionTo( GpuResidency::Resident, (uint8 *)0 );

                mMaxNumMipmaps = std::max( mMaxNumMipmaps, static_cast<uint8>( numMips ) );

                CompositorChannelVec channels;
                channels.push_back( actorData.reflectionTexture );
                actorData.workspace = mCompositorManager->addWorkspace(
                    mSceneManager, channels, actorData.reflectionCamera, workspaceName, false, 0 );
                actorData.isReserved = false;
                mActiveActorData.push_back( actorData );
            }
        }
    }
#endif
    //-----------------------------------------------------------------------------------
    PlanarReflectionActor *PlanarReflections::addActor( const PlanarReflectionActor &actorRef )
    {
        PlanarReflectionActor *actor = new PlanarReflectionActor( actorRef );
        mActors.push_back( actor );

        if( mCapacityActorsSoA < mActors.capacity() )
        {
            if( mActorsSoA )
            {
                OGRE_FREE_SIMD( mActorsSoA, MEMCATEGORY_SCENE_OBJECTS );
                mActorsSoA = 0;
                mCapacityActorsSoA = 0;
            }

            mCapacityActorsSoA = mActors.capacity();

            const size_t bytesRequired =
                sizeof( ArrayActorPlane ) *
                alignToNextMultiple<size_t>( mCapacityActorsSoA, ARRAY_PACKED_REALS ) /
                ARRAY_PACKED_REALS;
            mActorsSoA = reinterpret_cast<ArrayActorPlane *>(
                OGRE_MALLOC_SIMD( bytesRequired, MEMCATEGORY_SCENE_OBJECTS ) );

            mActors.pop_back();

            PlanarReflectionActorVec::const_iterator itor = mActors.begin();
            PlanarReflectionActorVec::const_iterator end = mActors.end();

            while( itor != end )
            {
                const size_t i = size_t( itor - mActors.begin() );
                ( *itor )->mIndex = i % ARRAY_PACKED_REALS;
                ( *itor )->mActorPlane = mActorsSoA + ( i / ARRAY_PACKED_REALS );
                ( *itor )->updateArrayActorPlane();
                ++itor;
            }

            mActors.push_back( actor );
        }

        actor->mIndex = ( mActors.size() - 1u ) % ARRAY_PACKED_REALS;
        actor->mActorPlane = mActorsSoA + ( ( mActors.size() - 1u ) / ARRAY_PACKED_REALS );

        actor->updateArrayActorPlane();

        return actor;
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::destroyActor( PlanarReflectionActor *actor )
    {
        PlanarReflectionActorVec::iterator itor = std::find( mActors.begin(), mActors.end(), actor );

        if( itor == mActors.end() )
        {
            OGRE_EXCEPT( Exception::ERR_ITEM_NOT_FOUND,
                         "Actor was not created by this PlanarReflections class "
                         "or was already destroyed!",
                         "PlanarReflections::destroyActor" );
        }

        if( actor->hasReservation() )
        {
            assert( actor->mCurrentBoundSlot < mActiveActorData.size() );
            mActiveActorData[actor->mCurrentBoundSlot].isReserved = false;
        }

        delete actor;

        efficientVectorRemove( mActors, itor );
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::destroyAllActors()
    {
        PlanarReflectionActorVec::iterator itor = mActors.begin();
        PlanarReflectionActorVec::iterator end = mActors.end();

        while( itor != end )
            delete *itor++;

        mActors.clear();

        ActiveActorDataVec::iterator itData = mActiveActorData.begin();
        ActiveActorDataVec::iterator enData = mActiveActorData.end();

        while( itData != enData )
        {
            itData->isReserved = false;
            ++itData;
        }
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::reserve( uint8 activeActorSlot, PlanarReflectionActor *actor )
    {
        assert( activeActorSlot < mActiveActorData.size() );
        assert( !mActiveActorData[activeActorSlot].isReserved &&
                "Slot is already reserved for another actor!" );
        assert( !actor->hasReservation() && "Actor already has a reservation!" );

        mActiveActorData[activeActorSlot].isReserved = true;
        actor->mHasReservation = true;
        actor->mCurrentBoundSlot = activeActorSlot;
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::releaseReservation( PlanarReflectionActor *actor )
    {
        assert( actor->hasReservation() && "Actor didn't have a reservation!" );
        assert( actor->mCurrentBoundSlot < mActiveActorData.size() &&
                "Actor has an invalid reservation! Does it belong to this PlanarReflections?" );
        assert( mActiveActorData[actor->mCurrentBoundSlot].isReserved &&
                "Slot wasn't reserved! Does the actor belong to this PlanarReflections?" );

        mActiveActorData[actor->mCurrentBoundSlot].isReserved = false;
        actor->mHasReservation = false;
        actor->mCurrentBoundSlot = 0xFF;
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::updateFlushedRenderables()
    {
        mUpdatingRenderablesHlms = true;

        TrackedRenderableArray::iterator itor = mTrackedRenderables.begin();
        TrackedRenderableArray::iterator end = mTrackedRenderables.end();

        while( itor != end )
        {
            TrackedRenderable &trackedRenderable = *itor;
            if( hasFlushPending( trackedRenderable.renderable ) )
            {
                HlmsDatablock *datablock = trackedRenderable.renderable->getDatablock();

                try
                {
                    Hlms *hlms = datablock->getCreator();

                    trackedRenderable.hlmsHashes[0] = trackedRenderable.renderable->getHlmsHash();

                    uint32 hash, casterHash;
                    hlms->calculateHashFor( trackedRenderable.renderable, hash, casterHash );
                    trackedRenderable.hlmsHashes[1] = hash;
                }
                catch( Exception &e )
                {
                    trackedRenderable.hlmsHashes[1] = trackedRenderable.hlmsHashes[0];

                    const String *datablockNamePtr = datablock->getNameStr();
                    String datablockName =
                        datablockNamePtr ? *datablockNamePtr : datablock->getName().getFriendlyText();

                    LogManager::getSingleton().logMessage( e.getFullDescription() );
                    LogManager::getSingleton().logMessage(
                        "Couldn't apply planar reflections change to datablock '" + datablockName +
                            "' for this renderable. Disabling reflections. "
                            "Check previous log messages to see if there's more information.",
                        LML_CRITICAL );
                }
            }

            ++itor;
        }

        mUpdatingRenderablesHlms = false;
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::addRenderable( const TrackedRenderable &trackedRenderable )
    {
#ifdef OGRE_BELIGHT_MINI
        if(mTrackedRenderables.size() >= ~0u)
            return;
        if(trackedRenderable.renderable->mTrackedRenderableIndex < mTrackedRenderables.size())
        {
            TrackedRenderableArray::iterator itor = mTrackedRenderables.begin() + trackedRenderable.renderable->mTrackedRenderableIndex;
            if(itor->renderable==trackedRenderable.renderable)
            {
                itor->planes.insert(itor->planes.end(), trackedRenderable.planes.begin(), trackedRenderable.planes.end());
                return;
            }
        }
        trackedRenderable.renderable->mTrackedRenderableIndex = mTrackedRenderables.size();
#endif
        assert( trackedRenderable.renderable->mCustomParameter == 0 );
        trackedRenderable.renderable->mCustomParameter = FlushPending;
        mTrackedRenderables.push_back( trackedRenderable );
        _notifyRenderableFlushedHlmsDatablock( trackedRenderable.renderable );
    }
#ifdef OGRE_BELIGHT_MINI
    //-----------------------------------------------------------------------------------
    void PlanarReflections::removeAllRenderables()
    {
        TrackedRenderableArray::iterator itor = mTrackedRenderables.begin();
        TrackedRenderableArray::iterator end  = mTrackedRenderables.end();
        while( itor != end )
        {
            if(itor->renderable)
            {
                if( !hasFlushPending( itor->renderable ) )
                {
                    //Restore the Hlms setting. If a flush is pending, then it's already up to date,
                    //and our TrackedRenderable::hlmsHashes[0] could be out of date.
                    itor->renderable->_setHlmsHashes( itor->hlmsHashes[0],
                                                      itor->renderable->getHlmsCasterHash() );
                }
                itor->renderable->mCustomParameter = 0;            
                itor->renderable->mTrackedRenderableIndex = ~0u;
            }
            ++itor;
        }
        mTrackedRenderables.clear();
    }
    void PlanarReflections::removeExternalTextures()
    {
        ActiveActorDataVec::iterator itor = mActiveActorData.begin();
        ActiveActorDataVec::iterator end  = mActiveActorData.end();
        for( ; itor != end; ++itor )
            itor->reflectionTexture = nullptr;
    }
#endif
    //-----------------------------------------------------------------------------------
    void PlanarReflections::removeRenderable( Renderable *renderable )
    {
#ifdef OGRE_BELIGHT_MINI
        if(renderable->mTrackedRenderableIndex == ~0u)
            return;
        if(renderable->mTrackedRenderableIndex < mTrackedRenderables.size())
        {
            TrackedRenderableArray::iterator itor = mTrackedRenderables.begin() + renderable->mTrackedRenderableIndex;
            if(itor->renderable==renderable)
            {
                if( !hasFlushPending( itor->renderable ) )
                {
                    //Restore the Hlms setting. If a flush is pending, then it's already up to date,
                    //and our TrackedRenderable::hlmsHashes[0] could be out of date.
                    itor->renderable->_setHlmsHashes( itor->hlmsHashes[0],
                                                      itor->renderable->getHlmsCasterHash() );
                }
                itor->renderable->mCustomParameter = 0;

                itor->renderable->mTrackedRenderableIndex = ~0u;
                itor = efficientVectorRemove( mTrackedRenderables, itor );
                if( itor != mTrackedRenderables.end() )
                    itor->renderable->mTrackedRenderableIndex = itor - mTrackedRenderables.begin();
                return;
            }
        }
#endif
        TrackedRenderableArray::iterator itor = mTrackedRenderables.begin();
        TrackedRenderableArray::iterator end = mTrackedRenderables.end();

        while( itor != end && itor->renderable != renderable )
            ++itor;

        assert( itor != end && "Item not found or already removed from this PlanarReflection!" );

        if( itor != end )
        {
            if( !hasFlushPending( itor->renderable ) )
            {
                // Restore the Hlms setting. If a flush is pending, then it's already up to date,
                // and our TrackedRenderable::hlmsHashes[0] could be out of date.
                itor->renderable->_setHlmsHashes( itor->hlmsHashes[0],
                                                  itor->renderable->getHlmsCasterHash() );
            }
            itor->renderable->mCustomParameter = 0;
#ifdef OGRE_BELIGHT_MINI
            itor->renderable->mTrackedRenderableIndex = ~0u;
            itor = efficientVectorRemove( mTrackedRenderables, itor );
            if( itor != mTrackedRenderables.end() )
                itor->renderable->mTrackedRenderableIndex = itor - mTrackedRenderables.begin();
#else
            efficientVectorRemove( mTrackedRenderables, itor );
#endif
        }
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::_notifyRenderableFlushedHlmsDatablock( Renderable *renderable )
    {
        if( !mUpdatingRenderablesHlms )
        {
            renderable->mCustomParameter = FlushPending;
            mAnyPendingFlushRenderable = true;
        }
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::beginFrame()
    {
        mLastCamera = 0;
        mLastAspectRatio = 0;

        mActiveActors.clear();
    }
    //-----------------------------------------------------------------------------------
#ifdef OGRE_BELIGHT_MINI
#ifdef DEBUG
//#define DEBUG_LOG_ACTIVE_ACTORS
#endif
    struct OrderPlanarReflectionActorsByDistanceToPointAndSize
    {
        Vector3 point;
        Camera* camera;
        Ogre::Matrix4 viewProjMatrix;
 
        OrderPlanarReflectionActorsByDistanceToPointAndSize( const Vector3 &p, Camera* cam ) :
            point( p ), camera(cam)
        {
            viewProjMatrix = cam->getProjectionMatrix() * cam->getViewMatrix();
        }
        
        void projectPlanarReflectionActorToScreenSize(const PlanarReflectionActor *actor) const
        {
            Vector3 vCorners[4];
            const Vector3& center = actor->getCenter();
            const Vector2& halfSize = actor->getHalfSize();
            const Quaternion& orientation = actor->getOrientation();
            Vector3 dy(orientation.yAxis() * halfSize.y);
            Vector3 dx(orientation.xAxis() * halfSize.x);
            vCorners[0] = center + dy - dx;
            vCorners[1] = center - dy - dx;
            vCorners[2] = center - dy + dx;
            vCorners[3] = center + dy + dx;
            for(int i=0; i<4; ++i)
            {
                Vector3& v = vCorners[i];
                Vector4 transformedVert(v);
                transformedVert = viewProjMatrix * transformedVert;
                transformedVert.w = std::max( transformedVert.w, Real(1e-6f) );
                transformedVert /= transformedVert.w;
                
                memcpy(v.ptr(), transformedVert.ptr(), sizeof(Real)*3);
                v.makeFloor( Vector3::UNIT_SCALE );
                v.makeCeil( -Vector3::UNIT_SCALE );
            }
            Vector2 vmin(vCorners[0].xy());
            Vector2 vmax = vmin;
            vmin.makeFloor(vCorners[1].xy());
            vmax.makeCeil(vCorners[1].xy());
            vmin.makeFloor(vCorners[2].xy());
            vmax.makeCeil(vCorners[2].xy());
            vmin.makeFloor(vCorners[3].xy());
            vmax.makeCeil(vCorners[3].xy());
            
            Ogre::Vector2 sz( vmax.x - vmin.x, vmax.y - vmin.y );
            actor->setScreenSize(sz);
        }

        bool operator () ( const PlanarReflectionActor *_l, const PlanarReflectionActor *_r ) const
        {
            if( _l->mActivationPriority == _r->mActivationPriority )
            {
                Real _lsize;
                Real _rsize;
                _lsize = _l->getScreenArea();
                _rsize = _r->getScreenArea();
                
                if(fabsf(_lsize-_rsize)<0.001f)
                    return _l->getSquaredDistanceTo( point ) < _r->getSquaredDistanceTo( point );
                                    
                return _lsize > _rsize;
            }

            return _l->mActivationPriority < _r->mActivationPriority;
        }
    };
#endif
    struct OrderPlanarReflectionActorsByDistanceToPoint
    {
        Vector3 point;

        OrderPlanarReflectionActorsByDistanceToPoint( const Vector3 &p ) : point( p ) {}

        bool operator()( const PlanarReflectionActor *_l, const PlanarReflectionActor *_r ) const
        {
            if( _l->mActivationPriority == _r->mActivationPriority )
                return _l->getSquaredDistanceTo( point ) < _r->getSquaredDistanceTo( point );

            return _l->mActivationPriority < _r->mActivationPriority;
        }
    };
    static bool OrderPlanarReflectionActorsByBindingSlot( const PlanarReflectionActor *_l,
                                                          const PlanarReflectionActor *_r )
    {
        return _l->getCurrentBoundSlot() < _r->getCurrentBoundSlot();
    };
    void PlanarReflections::update( Camera *camera, Real aspectRatio )
    {
        /*if( mLockCamera && camera != mLockCamera )
            return; //This is not the camera we are allowed to work with

        if( mLastCamera == camera &&
            mLastAspectRatio == camera->getAspectRatio() &&
            (!mLockCamera &&
             mLastCameraPos == camera->getDerivedPosition() &&
             mLastCameraRot == camera->getDerivedOrientation()) )
        {
            return;
        }*/

        if( mAnyPendingFlushRenderable )
        {
            updateFlushedRenderables();
            mAnyPendingFlushRenderable = false;
        }

        mActiveActors.clear();

        mLastAspectRatio = camera->getAspectRatio();
        mLastCameraPos = camera->getDerivedPosition();
        mLastCameraRot = camera->getDerivedOrientation();
        mLastCamera = camera;

        struct ArrayPlane
        {
            ArrayVector3 normal;
            ArrayReal negD;
        };

        // Cull actors against their master cameras (under SSE2, we cull 4 actors at the same time).
        // The culling algorithm is very similar to what is done in OgreForwardClustered.cpp which
        // is also described in
        // www.yosoygames.com.ar/wp/2016/12/frustum-vs-pyramid-intersection-also-frustum-vs-frustum/
        // However the difference is that it's like doing frustum vs frustum test; but the Actor
        // is a frustum with 0 height (thus flattened, hence... a plane) and we also have the
        // guarantee that the opposing sides normals are the same but negated; thus:
        //   rightPlane->normal = -leftPlane->normal
        //   farPlane->normal = -nearPlane->normal
        // Which is something a regular frustum won't guarantee. This saves us space storage.
        const size_t numActors = mActors.size();

        Camera *prevCameras[ARRAY_PACKED_REALS];
        memset( prevCameras, 0, sizeof( prevCameras ) );

        ArrayPlane frustums[6];
        ArrayVector3 worldSpaceCorners[8];

        ArrayActorPlane *RESTRICT_ALIAS actorsPlanes = mActorsSoA;

        {
            const Vector3 *corners = camera->getWorldSpaceCorners();
            for( int i = 0; i < 8; ++i )
                worldSpaceCorners[i].setAll( corners[i] );

            const Plane *planes = camera->getFrustumPlanes();
            for( int i = 0; i < 6; ++i )
            {
                frustums[i].normal.setAll( planes[i].normal );
                frustums[i].negD = Mathlib::SetAll( -planes[i].d );
            }
        }

        ArrayReal frustumFovCosLimit = Mathlib::SetAll(cosf(Math::HALF_PI - camera->getFOVy().valueRadians()*0.5f));

        for( size_t i = 0; i < numActors; i += ARRAY_PACKED_REALS )
        {
            ArrayMaskR mask;
            mask = BooleanMask4::getAllSetMask();
            
            ArrayReal dotResult0;
            ArrayVector3 actorPlaneNormals = actorsPlanes->planeNormals.zAxis();

            dotResult0 = frustums[0].normal.dotProduct( actorPlaneNormals );
            mask = Mathlib::And( mask, Mathlib::CompareLess( dotResult0, frustumFovCosLimit ) );

            // Test all 4 quad vertices against each of the 6 frustum planes.
            for( int k = 0; k < 6 && BooleanMask4::getScalarMask( mask ) != 0; ++k )
            {
                ArrayMaskR vertexMask = ARRAY_MASK_ZERO;
                ArrayReal dotResult;

                ArrayVector3 tangentDir, vertexPoint;

                tangentDir = actorsPlanes->planeNormals.yAxis() * actorsPlanes->xyHalfSize[1];
                vertexPoint = actorsPlanes->center + tangentDir;
                dotResult = frustums[k].normal.dotProduct( vertexPoint ) - frustums[k].negD;
                vertexMask =
                    Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );

                vertexPoint = actorsPlanes->center - tangentDir;
                dotResult = frustums[k].normal.dotProduct( vertexPoint ) - frustums[k].negD;
                vertexMask =
                    Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );

                tangentDir = actorsPlanes->planeNormals.xAxis() * actorsPlanes->xyHalfSize[0];
                vertexPoint = actorsPlanes->center + tangentDir;
                dotResult = frustums[k].normal.dotProduct( vertexPoint ) - frustums[k].negD;
                vertexMask =
                    Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );

                vertexPoint = actorsPlanes->center - tangentDir;
                dotResult = frustums[k].normal.dotProduct( vertexPoint ) - frustums[k].negD;
                vertexMask =
                    Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );

                mask = Mathlib::And( mask, vertexMask );
            }

            if( BooleanMask4::getScalarMask( mask ) != 0 )
            {
                // Test all 8 frustum corners against each of the 6 planes (5+1).
                ArrayVector3 actorPlaneNormal;
                ArrayReal actorPlaneNegD;
                ArrayReal dotResult;
                ArrayMaskR vertexMask;

                // Main plane (positive side)
                actorPlaneNormal = actorsPlanes->planeNormals.zAxis();
                actorPlaneNegD = actorsPlanes->planeNegD[0];
                vertexMask = ARRAY_MASK_ZERO;
                // Only test against the vertices from the near frustum. If they're all
                // in the negative side of this plane, then this plane is looking
                // away from the camera
                // for( int l=0; l<8; ++l )
                for( int l = 0; l < 4; ++l )
                {
                    dotResult = actorPlaneNormal.dotProduct( worldSpaceCorners[l] ) - actorPlaneNegD;
                    vertexMask =
                        Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );
                }
                mask = Mathlib::And( mask, vertexMask );

/*
                // Main plane (negative side)
                actorPlaneNormal = -actorPlaneNormal;
                actorPlaneNegD = -actorsPlanes->planeNegD[0];
                vertexMask = ARRAY_MASK_ZERO;
                for( int l = 0; l < 8; ++l )
                {
                    dotResult = actorPlaneNormal.dotProduct( worldSpaceCorners[l] ) - actorPlaneNegD;
                    vertexMask =
                        Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );
                }
                mask = Mathlib::And( mask, vertexMask );

                // North plane
                actorPlaneNormal = actorsPlanes->planeNormals.yAxis();
                actorPlaneNegD = actorsPlanes->planeNegD[1];
                vertexMask = ARRAY_MASK_ZERO;
                for( int l = 0; l < 8; ++l )
                {
                    dotResult = actorPlaneNormal.dotProduct( worldSpaceCorners[l] ) - actorPlaneNegD;
                    vertexMask =
                        Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );
                }
                mask = Mathlib::And( mask, vertexMask );

                // South plane
                actorPlaneNormal = -actorPlaneNormal;
                actorPlaneNegD = actorsPlanes->planeNegD[2];
                vertexMask = ARRAY_MASK_ZERO;
                for( int l = 0; l < 8; ++l )
                {
                    dotResult = actorPlaneNormal.dotProduct( worldSpaceCorners[l] ) - actorPlaneNegD;
                    vertexMask =
                        Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );
                }
                mask = Mathlib::And( mask, vertexMask );

                // East plane
                actorPlaneNormal = actorsPlanes->planeNormals.xAxis();
                actorPlaneNegD = actorsPlanes->planeNegD[3];
                vertexMask = ARRAY_MASK_ZERO;
                for( int l = 0; l < 8; ++l )
                {
                    dotResult = actorPlaneNormal.dotProduct( worldSpaceCorners[l] ) - actorPlaneNegD;
                    vertexMask =
                        Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );
                }
                mask = Mathlib::And( mask, vertexMask );

                // West plane
                actorPlaneNormal = -actorPlaneNormal;
                actorPlaneNegD = actorsPlanes->planeNegD[4];
                vertexMask = ARRAY_MASK_ZERO;
                for( int l = 0; l < 8; ++l )
                {
                    dotResult = actorPlaneNormal.dotProduct( worldSpaceCorners[l] ) - actorPlaneNegD;
                    vertexMask =
                        Mathlib::Or( vertexMask, Mathlib::CompareGreater( dotResult, ARRAY_REAL_ZERO ) );
                }
                mask = Mathlib::And( mask, vertexMask );
*/
            }

            const uint32 scalarMask = BooleanMask4::getScalarMask( mask );

            for( size_t j = 0; j < ARRAY_PACKED_REALS; ++j )
            {
                if( i + j < mActors.size() )
                {
                    if( IS_BIT_SET( j, scalarMask ) )
                        mActiveActors.push_back( mActors[i + j] );
                }
            }

            ++actorsPlanes;
        }

        const Vector3 camPos( camera->getDerivedPosition() );
#ifdef OGRE_BELIGHT_MINI
        mLastAspectRatio = aspectRatio;
        const Vector3 cameraDirection = camera->getDerivedDirection();
        OrderPlanarReflectionActorsByDistanceToPointAndSize actorsComparator(camPos, camera);
        for(auto aait = mActiveActors.begin(); aait!=mActiveActors.end(); ++aait)
            actorsComparator.projectPlanarReflectionActorToScreenSize(*aait);
        std::sort( mActiveActors.begin(), mActiveActors.end(), actorsComparator );
#ifdef DEBUG_LOG_ACTIVE_ACTORS
        size_t updatedPlaneReflections = 0u;
        LogManager::getSingleton().logMessage("PlanarReflections::update(): mActiveActors.size()==" + StringConverter::toString(mActiveActors.size()));
#endif
#else
        std::sort( mActiveActors.begin(), mActiveActors.end(),
                   OrderPlanarReflectionActorsByDistanceToPoint( camPos ) );
#endif
        mActiveActors.resize( std::min<size_t>( mActiveActors.size(), mMaxActiveActors ) );

        const Quaternion camRot( camera->getDerivedOrientation() );
        Real nearPlane = camera->getNearClipDistance();
        Real farPlane = camera->getFarClipDistance();
        Real focalLength = camera->getFocalLength();
        Radian fov = camera->getFOVy();

        {
            uint8 nextFreeActorData = 0;
            PlanarReflectionActorVec::iterator itor = mActiveActors.begin();
            PlanarReflectionActorVec::iterator end = mActiveActors.end();

            while( itor != end )
            {
                PlanarReflectionActor *actor = *itor;
                ActiveActorData *actorData = 0;
#ifdef OGRE_BELIGHT_MINI
                Ogre::Vector3 actor_vrect[4];
                if(actor->getScreenArea()>=0.1f && mSceneManager->isPlanarReflectionActorCustomCullVisible(camera, (size_t)actor) )
                {//use only big enought on screen actors
#ifdef DEBUG_LOG_ACTIVE_ACTORS
                     LogManager::getSingleton().logMessage("Use PlanarReflectionActor with screen size = " + StringConverter::toString( actor->getScreenArea() ) + " { " + StringConverter::toString( actor->getPlane().normal.x) + ", " + StringConverter::toString( actor->getPlane().normal.y) + ", " + StringConverter::toString( actor->getPlane().normal.z) + "; " + StringConverter::toString( actor->getPlane().d) + " }" );
#endif
#endif
                if( actor->hasReservation() )
                {
                    // Actor is bound to a specifc slot
                    const size_t idx = actor->mCurrentBoundSlot;
                    assert( idx < mActiveActorData.size() );
                    assert( mActiveActorData[idx].isReserved &&
                            "Actor says he has a reservation on this slot, but the slot disagrees." );
                    actorData = &mActiveActorData[idx];
                }
                else
                {
                    while( nextFreeActorData < mActiveActorData.size() &&
                           mActiveActorData[nextFreeActorData].isReserved )
                    {
                        ++nextFreeActorData;
                    }

                    if( nextFreeActorData < mActiveActorData.size() )
                    {
                        actorData = &mActiveActorData[nextFreeActorData];
                        // Grab whatever non-reserved slot we can get.
                        actor->mCurrentBoundSlot = nextFreeActorData;
                        ++nextFreeActorData;
                    }
                }
#ifdef OGRE_BELIGHT_MINI
                }
#ifdef DEBUG
                else
                    LogManager::getSingleton().logMessage("Skipped PlanarReflectionActor with screen size = " + StringConverter::toString( actor->getScreenArea() ) + " { " + StringConverter::toString( actor->getPlane().normal.x) + ", " + StringConverter::toString( actor->getPlane().normal.y) + ", " + StringConverter::toString( actor->getPlane().normal.z) + "; " + StringConverter::toString( actor->getPlane().d) + " }" );
#endif
#endif

                if( actorData )
                {
                    actorData->workspace->setEnabled( true );
                    actorData->reflectionCamera->setPosition( camPos );
                    actorData->reflectionCamera->setOrientation( camRot );
                    actorData->reflectionCamera->setNearClipDistance( nearPlane );
                    actorData->reflectionCamera->setFarClipDistance( farPlane );
                    actorData->reflectionCamera->setAspectRatio( aspectRatio );
                    actorData->reflectionCamera->setFocalLength( focalLength );
                    actorData->reflectionCamera->setFOVy( fov );
#ifdef OGRE_BELIGHT_MINI
                    Plane p = actor->mPlane;
                    p.normal.normalise();
                    Vector3 v = actor->mCenter - (p.normal * p.getDistance(actor->mCenter));
                    p.redefine(p.normal, p.normal * 0.001f + v); //move reflection plane forward to one mm
                    actorData->reflectionCamera->enableReflection( p );
#else
                    actorData->reflectionCamera->enableReflection( actor->mPlane );
#endif

                    if( camera->getFrustumExtentsManuallySet() )
                    {
                        Ogre::Vector4 frustumExtents;
                        camera->getFrustumExtents( frustumExtents.x, frustumExtents.y, frustumExtents.z,
                                                   frustumExtents.w );
                        actorData->reflectionCamera->setFrustumExtents(
                            frustumExtents.x, frustumExtents.y, frustumExtents.z, frustumExtents.w,
                            Ogre::FET_PROJ_PLANE_POS );
                    }

                    ++itor;
                }
                else
                {
                    // If we're here we don't have a reservation and there are no
                    // more free slots for us to grab. We can't activate this actor.
                    const ptrdiff_t idx = itor - mActiveActors.begin();
                    mActiveActors.erase( itor );
                    itor = mActiveActors.begin() + idx;
                    end = mActiveActors.end();
                }
            }
        }

        TrackedRenderableArray::const_iterator itTracked = mTrackedRenderables.begin();
        TrackedRenderableArray::const_iterator enTracked = mTrackedRenderables.end();

        while( itTracked != enTracked )
        {
            if( itTracked->movableObject->getVisible() )
            {
#ifdef OGRE_BELIGHT_MINI
                uint8 bestActorIdx = mMaxActiveActors;
                Real bestCosAngle = -1;
                Real bestSqDistance = std::numeric_limits<Real>::max();

                const Matrix4 &fullTransform = itTracked->movableObject->_getParentNodeFullTransform();
                Matrix3 rotMat3x3;
                fullTransform.extract3x3Matrix( rotMat3x3 );
                Vector3 reflNormal;
                
                assert(itTracked->planes.size()>0);
                std::vector<PlaneWithCenter>::const_iterator itp = itTracked->planes.begin();
                std::vector<PlaneWithCenter>::const_iterator itp_end = itTracked->planes.end();
                Ogre::Vector3 curNormal;
                Real curDot;
                Real bestPlaneDotWithCameraLook = 1.0f;
                Vector3 rendCenter;
                Vector3 curCenter;
                std::vector<PlaneWithCenter>::const_iterator itp_best = itp_end;
                Plane curPlane;
                for(; itp != itp_end; ++itp)
                {
                    curNormal = rotMat3x3 * itp->planeNormal;
                    curNormal.normalise();
                    curCenter = fullTransform * itp->planeCenter;
                    curPlane.redefine(curNormal, curCenter);
                    if(curPlane.getSide(camPos)==Plane::POSITIVE_SIDE)
                    {
                        curDot = cameraDirection.dotProduct(curNormal);
                        if(curDot<bestPlaneDotWithCameraLook)
                        {
                            bestPlaneDotWithCameraLook = curDot;
                            itp_best = itp;
                            reflNormal = curNormal;
                            rendCenter = curCenter;
                        }
                    }
                }
                if(itp_best == itp_end)
                {
                    itTracked->renderable->mCustomParameter = InactiveActor;
                    itTracked->renderable->_setHlmsHashes( itTracked->hlmsHashes[0],
                                                           itTracked->renderable->getHlmsCasterHash() );
                    ++itTracked;
                    continue;
                }

                PlanarReflectionActorVec::const_iterator itor = mActiveActors.begin();
                PlanarReflectionActorVec::const_iterator end  = mActiveActors.end();

                const Real cos5 = 0.9962f; //5 degree
                const Real delta5 = 1.0f - 0.9962f;

                while( itor != end )
                {
                    PlanarReflectionActor *actor = *itor;
                    const Real cosAngle = actor->getNormal().dotProduct( reflNormal );

                    if( cosAngle >= cos5 &&
                        (cosAngle >= bestCosAngle ||
                        Math::Abs(cosAngle - bestCosAngle) < delta5) )
                    {
                        Real sqDistance = actor->getSquaredDistanceTo( rendCenter );
                        if( sqDistance < mMaxSqDistance && sqDistance <= bestSqDistance )
                        {
                            bestActorIdx = (*itor)->mCurrentBoundSlot;
                            bestSqDistance = sqDistance;
                            bestCosAngle = cosAngle;
                        }
                    }

                    ++itor;
                }
#else
                const Matrix4 &fullTransform = itTracked->movableObject->_getParentNodeFullTransform();
                Matrix3 rotMat3x3;
                fullTransform.extract3x3Matrix( rotMat3x3 );
                Vector3 reflNormal = rotMat3x3 * itTracked->reflNormal;
                const Vector3 rendCenter = fullTransform * itTracked->renderableCenter;

                // Undo any scale
                reflNormal.normalise();

                uint8 bestActorIdx = mMaxActiveActors;
                Real bestCosAngle = -1;
                Real bestSqDistance = std::numeric_limits<Real>::max();

                PlanarReflectionActorVec::const_iterator itor = mActiveActors.begin();
                PlanarReflectionActorVec::const_iterator end = mActiveActors.end();

                while( itor != end )
                {
                    PlanarReflectionActor *actor = *itor;
                    const Real cosAngle = actor->getNormal().dotProduct( reflNormal );

                    const Real cos20 = 0.939692621f;

                    if( cosAngle >= cos20 &&
                        ( cosAngle >= bestCosAngle ||
                          Math::Abs( cosAngle - bestCosAngle ) < Real( 0.060307379f ) ) )
                    {
                        Real sqDistance = actor->getSquaredDistanceTo( rendCenter );
                        if( sqDistance < mMaxSqDistance && sqDistance <= bestSqDistance )
                        {
                            bestActorIdx = ( *itor )->mCurrentBoundSlot;
                            bestSqDistance = sqDistance;
                            bestCosAngle = cosAngle;
                        }
                    }

                    ++itor;
                }
#endif
                if( bestActorIdx < mMaxActiveActors )
                {
                    itTracked->renderable->mCustomParameter = ( UseActiveActor | bestActorIdx );
                    itTracked->renderable->_setHlmsHashes( itTracked->hlmsHashes[1],
                                                           itTracked->renderable->getHlmsCasterHash() );
                }
                else
                {
                    itTracked->renderable->mCustomParameter = InactiveActor;
                    itTracked->renderable->_setHlmsHashes( itTracked->hlmsHashes[0],
                                                           itTracked->renderable->getHlmsCasterHash() );
                }
            }

            ++itTracked;
        }

        // Now force mActiveActors & mActiveActorData to match, that means:
        //   mActiveActors[idx]->getCurrentBoundSlot() == idx.

        // mActiveActors may not be sorted due to reserved slots (i.e. mActiveActors[5]
        // may have had the reservation on mActiveActorData[0]
        std::sort( mActiveActors.begin(), mActiveActors.end(),
                   OrderPlanarReflectionActorsByBindingSlot );

        // Now fill in the gaps. Due to the reservation system, it's possible we have
        //   0, 1, 3, 4
        // because slot 2 is reserved and actor was not activated. If this is the case, fill
        // in a dummy (note that dummies will have dummy->mCurrentBoundSlot = 0xFF because
        // we only reuse the same dummy pointer for all missing slots). Afterwards it will read:
        //   0, 1, 255(dummy), 3, 4
        // which for all purposes we need, it's the same as:
        //   0, 1, 2, 3, 4
        size_t lastIdx = std::numeric_limits<size_t>::max();
        PlanarReflectionActorVec::iterator itActor = mActiveActors.begin();
        PlanarReflectionActorVec::iterator enActor = mActiveActors.end();
        while( itActor != enActor )
        {
            if( lastIdx + 1u != ( *itActor )->mCurrentBoundSlot )
            {
                const size_t diff = ( *itActor )->mCurrentBoundSlot - ( lastIdx + 1u );
                const ptrdiff_t newIdx = itActor - mActiveActors.begin() + ptrdiff_t( diff );
                mActiveActors.insert( itActor, diff, &mDummyActor );
                itActor = mActiveActors.begin() + newIdx;
                enActor = mActiveActors.end();
                lastIdx += size_t( diff );
            }

            ++lastIdx;
            ++itActor;
        }

        {
            ActiveActorDataVec::const_iterator itor = mActiveActorData.begin();
            ActiveActorDataVec::const_iterator end = mActiveActorData.end();

            while( itor != end )
            {
                const ActiveActorData &actorData = *itor;
                if( actorData.workspace->getEnabled() )
                {
#ifdef OGRE_BELIGHT_MINI
#ifdef DEBUG_LOG_ACTIVE_ACTORS
                    ++updatedPlaneReflections;
#endif
#endif
                    actorData.workspace->_update();
                    actorData.workspace->setEnabled( false );
                }

                ++itor;
            }
#ifdef OGRE_BELIGHT_MINI
#ifdef DEBUG_LOG_ACTIVE_ACTORS
            LogManager::getSingleton().logMessage("Updated " + StringConverter::toString(updatedPlaneReflections) + " PlaneReflections");
#endif
#endif
            
        }
    }
    //-----------------------------------------------------------------------------------
    size_t PlanarReflections::getConstBufferSize() const
    {
        return ( 4u * mMaxActiveActors + 4u * 4u + 4u ) * sizeof( float );
    }
    //-----------------------------------------------------------------------------------
    void PlanarReflections::fillConstBufferData( TextureGpu *renderTarget, const Camera *camera,
                                                 const Matrix4 &projectionMatrix,
                                                 float *RESTRICT_ALIAS passBufferPtr ) const
    {
        const Matrix4 viewMatrix = camera->getViewMatrix( true );
        Matrix3 viewMat3x3;
        viewMatrix.extract3x3Matrix( viewMat3x3 );

        PlanarReflectionActorVec::const_iterator itor = mActiveActors.begin();
        PlanarReflectionActorVec::const_iterator end = mActiveActors.end();

        while( itor != end )
        {
            const Plane &plane = ( *itor )->mPlane;

            // We need to send the plane data for PBS (which will
            // be processed in pixel shader)in view space.
            const Vector3 viewSpacePointInPlane = viewMatrix * ( plane.normal * -plane.d );
            const Vector3 viewSpaceNormal = viewMat3x3 * plane.normal;

            Plane planeVS( viewSpaceNormal, viewSpacePointInPlane );

            *passBufferPtr++ = static_cast<float>( planeVS.normal.x );
            *passBufferPtr++ = static_cast<float>( planeVS.normal.y );
            *passBufferPtr++ = static_cast<float>( planeVS.normal.z );
            *passBufferPtr++ = static_cast<float>( planeVS.d );

            ++itor;
        }

        memset( passBufferPtr, 0, ( mMaxActiveActors - mActiveActors.size() ) * 4u * sizeof( float ) );
        passBufferPtr += ( mMaxActiveActors - mActiveActors.size() ) * 4u;

        Matrix4 reflProjMat = PROJECTIONCLIPSPACE2DTOIMAGESPACE_PERSPECTIVE * projectionMatrix;
        for( size_t i = 0; i < 16; ++i )
            *passBufferPtr++ = (float)reflProjMat[0][i];

        *passBufferPtr++ = static_cast<float>( mInvMaxDistance );
        *passBufferPtr++ = 1.0f;
        *passBufferPtr++ = 1.0f;
        *passBufferPtr++ = 1.0f;
    }
    //-----------------------------------------------------------------------------------
    TextureGpu *PlanarReflections::getTexture( uint8 actorIdx ) const
    {
        if( actorIdx >= mActiveActorData.size() )
            return 0;
        return mActiveActorData[actorIdx].reflectionTexture;
    }
    //-----------------------------------------------------------------------------------
    bool PlanarReflections::cameraMatches( const Camera *camera )
    {
#ifdef OGRE_BELIGHT_MINI
        return !camera->isReflected() &&
               fabs(mLastAspectRatio - camera->getAspectRatio())<=0.001f &&
               mLastCameraPos == camera->getDerivedPosition() &&
               mLastCameraRot == camera->getDerivedOrientation();
#else
        return !camera->isReflected() && mLastAspectRatio == camera->getAspectRatio() &&
               mLastCameraPos == camera->getDerivedPosition() &&
               mLastCameraRot == camera->getDerivedOrientation();
#endif
    }
    //-----------------------------------------------------------------------------------
    bool PlanarReflections::_isUpdatingRenderablesHlms() const { return mUpdatingRenderablesHlms; }
    //-----------------------------------------------------------------------------------
    bool PlanarReflections::hasPlanarReflections( const Renderable *renderable ) const
    {
        return renderable->mCustomParameter & UseActiveActor ||
               renderable->mCustomParameter == FlushPending ||
               renderable->mCustomParameter == InactiveActor;
    }
    //-----------------------------------------------------------------------------------
    bool PlanarReflections::hasFlushPending( const Renderable *renderable ) const
    {
        return renderable->mCustomParameter == FlushPending;
    }
    //-----------------------------------------------------------------------------------
    bool PlanarReflections::hasActiveActor( const Renderable *renderable ) const
    {
        return ( renderable->mCustomParameter & UseActiveActor ) != 0;
    }
}  // namespace Ogre
