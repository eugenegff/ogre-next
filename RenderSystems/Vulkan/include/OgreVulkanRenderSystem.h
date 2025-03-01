/*
-----------------------------------------------------------------------------
This source file is part of OGRE-Next
(Object-oriented Graphics Rendering Engine)
For the latest info, see http://www.ogre3d.org

Copyright (c) 2000-present Torus Knot Software Ltd

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

#ifndef _OgreVulkanRenderSystem_H_
#define _OgreVulkanRenderSystem_H_

#include "OgreVulkanPrerequisites.h"

#include "OgreHlmsPso.h"
#include "OgreRenderSystem.h"
#include "OgreVulkanGlobalBindingTable.h"
#include "OgreVulkanPixelFormatToShaderType.h"
#include "OgreVulkanProgram.h"

#include "OgreVulkanRenderPassDescriptor.h"
#include "Vao/OgreVulkanConstBufferPacked.h"

namespace Ogre
{
    namespace v1
    {
        class HardwareBufferManager;
    }

    struct VulkanHlmsPso;
    class VulkanSupport;

    /**
       Implementation of Vulkan as a rendering system.
    */
    class _OgreVulkanExport VulkanRenderSystem final : public RenderSystem
    {
        bool mInitialized;
        v1::HardwareBufferManager *mHardwareBufferManager;

        VulkanPixelFormatToShaderType mPixelFormatToShaderType;

        VkBuffer mIndirectBuffer;
        unsigned char *mSwIndirectBufferPtr;

        VulkanGpuProgramManager *mShaderManager;
        VulkanProgramFactory *mVulkanProgramFactory0;
        VulkanProgramFactory *mVulkanProgramFactory1;
        VulkanProgramFactory *mVulkanProgramFactory2;
        VulkanProgramFactory *mVulkanProgramFactory3;

        VkInstance mVkInstance;
        VulkanSupport *mVulkanSupport;

        // TODO: AutoParamsBuffer probably belongs to MetalDevice (because it's per device?)
        typedef vector<ConstBufferPacked *>::type ConstBufferPackedVec;
        ConstBufferPackedVec mAutoParamsBuffer;
        size_t mAutoParamsBufferIdx;
        uint8 *mCurrentAutoParamsBufferPtr;
        size_t mCurrentAutoParamsBufferSpaceLeft;
        size_t mHistoricalAutoParamsSize[60];

        // For v1 rendering.
        v1::IndexData *mCurrentIndexBuffer;
        v1::VertexData *mCurrentVertexBuffer;
        VkPrimitiveTopology mCurrentPrimType;

        VulkanDevice *mActiveDevice;

        VulkanDevice *mDevice;

        VulkanCache *mCache;

        HlmsPso const *mPso;
        HlmsComputePso const *mComputePso;

        uint32_t mStencilRefValue;
        bool mStencilEnabled;

        bool mTableDirty;
        bool mComputeTableDirty;
        VulkanGlobalBindingTable mGlobalTable;
        VulkanGlobalBindingTable mComputeTable;
        // Vulkan requires a valid handle when updating descriptors unless nullDescriptor is present
        // So we just use a dummy. The dummy texture we get it from TextureGpuManager which needs
        // to create some anyway for different reasons
        ConstBufferPacked *mDummyBuffer;
        TexBufferPacked *mDummyTexBuffer;
        VkImageView mDummyTextureView;
        VkSampler mDummySampler;

        // clang-format off
        VulkanFrameBufferDescMap    mFrameBufferDescMap;
        VulkanFlushOnlyDescMap      mFlushOnlyDescMap;
        uint32                      mEntriesToFlush;
        bool                        mVpChanged;
        bool                        mInterruptedRenderCommandEncoder;
        // clang-format on

        bool mValidationError;

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        bool mHasWin32Support;
#elif OGRE_PLATFORM == OGRE_PLATFORM_ANDROID
        bool mHasAndroidSupport;
#else
        bool mHasXcbSupport;
#endif
#if OGRE_DEBUG_MODE >= OGRE_DEBUG_HIGH
        bool mHasValidationLayers;
#endif

        PFN_vkCreateDebugReportCallbackEXT CreateDebugReportCallback;
        PFN_vkDestroyDebugReportCallbackEXT DestroyDebugReportCallback;
        VkDebugReportCallbackEXT mDebugReportCallback;

#if OGRE_DEBUG_MODE >= OGRE_DEBUG_MEDIUM
        PFN_vkCmdBeginDebugUtilsLabelEXT CmdBeginDebugUtilsLabelEXT;
        PFN_vkCmdEndDebugUtilsLabelEXT CmdEndDebugUtilsLabelEXT;
#endif

        /// Declared here to avoid constant reallocations
        FastArray<VkImageMemoryBarrier> mImageBarriers;

        void addInstanceDebugCallback();

        /// Creates a dummy VkRenderPass for use in PSO creation
        VkRenderPass getVkRenderPass( HlmsPassPso passPso, uint8 &outMrtCount );

        void bindDescriptorSet() const;

        void flushRootLayout();
        void flushRootLayoutCS();

    public:
        VulkanRenderSystem();
        ~VulkanRenderSystem() override;

        void shutdown() override;

        const String &getName() const override;
        const String &getFriendlyName() const override;
        void refreshConfig();
        void initConfigOptions();
        ConfigOptionMap &getConfigOptions() override;
        void setConfigOption( const String &name, const String &value ) override;
        const char *getPriorityConfigOption( size_t idx ) const override;
        size_t getNumPriorityConfigOptions() const override;

        HardwareOcclusionQuery *createHardwareOcclusionQuery() override;

        String validateConfigOptions() override;

        RenderSystemCapabilities *createRenderSystemCapabilities() const override;

        void resetAllBindings();

        void reinitialise() override;

        void initializeVkInstance();

        VkInstance getVkInstance() const { return mVkInstance; }

        Window *_initialise( bool autoCreateWindow,
                             const String &windowTitle = "OGRE Render Window" ) override;

        Window *_createRenderWindow( const String &name, uint32 width, uint32 height, bool fullScreen,
                                     const NameValuePairList *miscParams = 0 ) override;

        String getErrorDescription( long errorNumber ) const override;

        void _useLights( const LightList &lights, unsigned short limit ) override;
        void _setWorldMatrix( const Matrix4 &m ) override;
        void _setViewMatrix( const Matrix4 &m ) override;
        void _setProjectionMatrix( const Matrix4 &m ) override;

        void _setSurfaceParams( const ColourValue &ambient, const ColourValue &diffuse,
                                const ColourValue &specular, const ColourValue &emissive, Real shininess,
                                TrackVertexColourType tracking = TVC_NONE ) override;
        void _setPointSpritesEnabled( bool enabled ) override;
        void _setPointParameters( Real size, bool attenuationEnabled, Real constant, Real linear,
                                  Real quadratic, Real minSize, Real maxSize ) override;

        void flushUAVs();

        void _setParamBuffer( GpuProgramType shaderStage, const VkDescriptorBufferInfo &bufferInfo );
        void _setConstBuffer( size_t slot, const VkDescriptorBufferInfo &bufferInfo );
        void _setConstBufferCS( size_t slot, const VkDescriptorBufferInfo &bufferInfo );
        void _setTexBuffer( size_t slot, VkBufferView bufferView );
        void _setTexBufferCS( size_t slot, VkBufferView bufferView );
        void _setReadOnlyBuffer( size_t slot, const VkDescriptorBufferInfo &bufferInfo );

        void _setCurrentDeviceFromTexture( TextureGpu *texture ) override;
        void _setTexture( size_t unit, TextureGpu *texPtr, bool bDepthReadOnly ) override;
        void _setTextures( uint32 slotStart, const DescriptorSetTexture *set,
                           uint32 hazardousTexIdx ) override;
        void _setTextures( uint32 slotStart, const DescriptorSetTexture2 *set ) override;
        void _setSamplers( uint32 slotStart, const DescriptorSetSampler *set ) override;
        void _setTexturesCS( uint32 slotStart, const DescriptorSetTexture *set ) override;
        void _setTexturesCS( uint32 slotStart, const DescriptorSetTexture2 *set ) override;
        void _setSamplersCS( uint32 slotStart, const DescriptorSetSampler *set ) override;
        void _setUavCS( uint32 slotStart, const DescriptorSetUav *set ) override;

        void _setTextureCoordCalculation( size_t unit, TexCoordCalcMethod m,
                                          const Frustum *frustum = 0 ) override;
        void _setTextureBlendMode( size_t unit, const LayerBlendModeEx &bm ) override;
        void _setTextureMatrix( size_t unit, const Matrix4 &xform ) override;

        void _setIndirectBuffer( IndirectBufferPacked *indirectBuffer ) override;

        VulkanFrameBufferDescMap &_getFrameBufferDescMap() { return mFrameBufferDescMap; }
        VulkanFlushOnlyDescMap &_getFlushOnlyDescMap() { return mFlushOnlyDescMap; }
        RenderPassDescriptor *createRenderPassDescriptor() override;

        void _hlmsComputePipelineStateObjectCreated( HlmsComputePso *newPso ) override;
        void _hlmsComputePipelineStateObjectDestroyed( HlmsComputePso *newPso ) override;

        void setStencilBufferParams( uint32 refValue, const StencilParams &stencilParams ) override;

        void _beginFrame() override;
        void _endFrame() override;
        void _endFrameOnce() override;

        void _setHlmsSamplerblock( uint8 texUnit, const HlmsSamplerblock *Samplerblock ) override;
        void _setPipelineStateObject( const HlmsPso *pso ) override;
        void _setComputePso( const HlmsComputePso *pso ) override;

        VertexElementType getColourVertexElementType() const override;

        void _dispatch( const HlmsComputePso &pso ) override;

        void _setVertexArrayObject( const VertexArrayObject *vao ) override;
        void flushDescriptorState(
            VkPipelineBindPoint pipeline_bind_point, const VulkanConstBufferPacked &constBuffer,
            const size_t bindOffset, const size_t bytesToWrite,
            const unordered_map<unsigned, VulkanConstantDefinitionBindingParam>::type &shaderBindings );

        void _render( const CbDrawCallIndexed *cmd ) override;
        void _render( const CbDrawCallStrip *cmd ) override;
        void bindDescriptorSet( VulkanVaoManager *&vaoManager );
        void _renderEmulated( const CbDrawCallIndexed *cmd ) override;
        void _renderEmulated( const CbDrawCallStrip *cmd ) override;

        void _setRenderOperation( const v1::CbRenderOp *cmd ) override;
        void _render( const v1::CbDrawCallIndexed *cmd ) override;
        void _render( const v1::CbDrawCallStrip *cmd ) override;

        void _render( const v1::RenderOperation &op ) override;

        void bindGpuProgramParameters( GpuProgramType gptype, GpuProgramParametersSharedPtr params,
                                       uint16 variabilityMask ) override;
        void bindGpuProgramPassIterationParameters( GpuProgramType gptype ) override;

        void clearFrameBuffer( RenderPassDescriptor *renderPassDesc, TextureGpu *anyTarget,
                               uint8 mipLevel ) override;

        Real getHorizontalTexelOffset() override;
        Real getVerticalTexelOffset() override;
        Real getMinimumDepthInputValue() override;
        Real getMaximumDepthInputValue() override;

        void preExtraThreadsStarted() override;
        void postExtraThreadsStarted() override;
        void registerThread() override;
        void unregisterThread() override;
        unsigned int getDisplayMonitorCount() const override { return 1; }

        const PixelFormatToShaderType *getPixelFormatToShaderType() const override;

        void flushCommands() override;

        void beginProfileEvent( const String &eventName ) override;
        void endProfileEvent() override;
        void markProfileEvent( const String &event ) override;

        void debugAnnotationPush( const String &event ) override;
        void debugAnnotationPop() override;

        void initGPUProfiling() override;
        void deinitGPUProfiling() override;
        void beginGPUSampleProfile( const String &name, uint32 *hashCache ) override;
        void endGPUSampleProfile( const String &name ) override;

        bool hasAnisotropicMipMapFilter() const override { return true; }

        void setClipPlanesImpl( const PlaneList &clipPlanes ) override;
        void initialiseFromRenderSystemCapabilities( RenderSystemCapabilities *caps,
                                                     Window *primary ) override;

        void beginRenderPassDescriptor( RenderPassDescriptor *desc, TextureGpu *anyTarget,
                                        uint8 mipLevel, const Vector4 *viewportSizes,
                                        const Vector4 *scissors, uint32 numViewports,
                                        bool overlaysEnabled, bool warnIfRtvWasFlushed ) override;
        void executeRenderPassDescriptorDelayedActions( bool officialCall );
        void executeRenderPassDescriptorDelayedActions() override;
        inline void endRenderPassDescriptor( bool isInterruptingRender );
        void endRenderPassDescriptor() override;

        TextureGpu *createDepthBufferFor( TextureGpu *colourTexture, bool preferDepthTexture,
                                          PixelFormatGpu depthBufferFormat, uint16 poolId ) override;

        void notifySwapchainCreated( VulkanWindow *window );
        void notifySwapchainDestroyed( VulkanWindow *window );

        void notifyRenderTextureNonResident( VulkanTextureGpu *texture );

        void endCopyEncoder() override;
        void executeResourceTransition( const ResourceTransitionArray &rstCollection ) override;

        void _hlmsPipelineStateObjectCreated( HlmsPso *newPso ) override;
        void _hlmsPipelineStateObjectDestroyed( HlmsPso *pos ) override;
        void _hlmsMacroblockCreated( HlmsMacroblock *newBlock ) override;
        void _hlmsMacroblockDestroyed( HlmsMacroblock *block ) override;
        void _hlmsBlendblockCreated( HlmsBlendblock *newBlock ) override;
        void _hlmsBlendblockDestroyed( HlmsBlendblock *block ) override;
        void _hlmsSamplerblockCreated( HlmsSamplerblock *newBlock ) override;
        void _hlmsSamplerblockDestroyed( HlmsSamplerblock *block ) override;
        void _descriptorSetTextureCreated( DescriptorSetTexture *newSet ) override;
        void _descriptorSetTextureDestroyed( DescriptorSetTexture *set ) override;
        void _descriptorSetTexture2Created( DescriptorSetTexture2 *newSet ) override;
        void _descriptorSetTexture2Destroyed( DescriptorSetTexture2 *set ) override;
        void _descriptorSetSamplerCreated( DescriptorSetSampler *newSet ) override;
        void _descriptorSetSamplerDestroyed( DescriptorSetSampler *set ) override;
        void _descriptorSetUavCreated( DescriptorSetUav *newSet ) override;
        void _descriptorSetUavDestroyed( DescriptorSetUav *set ) override;

        SampleDescription validateSampleDescription( const SampleDescription &sampleDesc,
                                                     PixelFormatGpu format ) override;
        VulkanDevice *getVulkanDevice() const { return mDevice; }
        void _notifyDeviceStalled();

        void _notifyActiveEncoderEnded( bool callEndRenderPassDesc );
        void _notifyActiveComputeEnded();

        void debugCallback();

        bool isSameLayout( ResourceLayout::Layout a, ResourceLayout::Layout b, const TextureGpu *texture,
                           bool bIsDebugCheck ) const override;
    };
}  // namespace Ogre

#endif
