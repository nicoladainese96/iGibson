from gibson2.render.mesh_renderer.mesh_renderer_cpu import MeshRenderer, MeshRendererSettings
from gibson2.utils.mesh_util import lookat
from gibson2 import assets_path
import json
from json_minify import json_minify
import numpy as np
import os
import time


class VrOverlayBase(object):
    """
    Base class representing a VR overlay. Use one of the subclasses to create a specific overlay.
    """
    def __init__(self, overlay_name, renderer, width=1, pos=[0, 0, -1]):
        """
        :param overlay_name: the name of the overlay - must be a unique string
        :param renderer: instance of MeshRendererVR
        :param width: width of the overlay quad in meters
        :param pos: location of overlay quad - x is left, y is up and z is away from camera in VR headset space
        """
        self.overlay_name = overlay_name
        self.renderer = renderer
        self.width = width
        self.pos = pos
        # Note: overlay will only be instantiated in subclasses
    
    def set_overlay_show_state(self, show):
        """
        Sets show state of an overlay
        :param state: True to show, False to hide
        """
        self.show_state = show
        if self.show_state:
            self.renderer.vrsys.showOverlay(self.overlay_name)
        else:
            self.renderer.vrsys.hideOverlay(self.overlay_name)
    
    def get_overlay_show_state(self):
        """
        Returns show state of an overlay
        """
        return self.show_state


class VrHUDOverlay(VrOverlayBase):
    """
    Class that renders all Text objects with render_to_tex=True to a Vr overlay. Can be used for rendering user instructions, for example.
    Text should not be rendered to the non-VR screen, as it will then appear as part of the VR image!
    There should only be one of these VrHUDOverlays per scene, as it will render all text. HUD stands for heads-up-display.
    """
    def __init__(self, 
                 overlay_name, 
                 renderer, 
                 width=1, 
                 pos=[0, 0, -1]):
        """
        :param overlay_name: the name of the overlay - must be a unique string
        :param renderer: instance of MeshRendererVR
        :param width: width of the overlay quad in meters
        :param pos: location of overlay quad - x is left, y is up and z is away from camera in VR headset space
        """
        super().__init__(overlay_name, renderer, width=width, pos=pos)
        self.renderer.vrsys.createOverlay(self.overlay_name, self.width, self.pos[0], self.pos[1], self.pos[2], '')

    def refresh_text(self):
        """
        Updates VR overlay texture with new text.
        """
        # Skip update if there is no text to render
        if len(self.renderer.texts) == 0:
            return
        rtex = self.renderer.text_manager.get_render_tex()
        self.renderer.vrsys.updateOverlayTexture(self.overlay_name, rtex)

class VrStaticImageOverlay(VrOverlayBase):
    """
    Class that renders a static image to the VR overlay a single time.
    """
    def __init__(self, 
                 overlay_name, 
                 renderer, 
                 image_fpath,
                 width=1, 
                 pos=[0, 0, -1]):
        """
        :param overlay_name: the name of the overlay - must be a unique string
        :param renderer: instance of MeshRendererVR
        :param image_fpath: path to image to render to overlay
        :param width: width of the overlay quad in meters
        :param pos: location of overlay quad - x is left, y is up and z is away from camera in VR headset space
        """
        super().__init__(overlay_name, renderer, width=width, pos=pos)
        self.image_fpath = image_fpath
        self.renderer.vrsys.createOverlay(self.overlay_name, self.width, self.pos[0], self.pos[1], self.pos[2], self.image_fpath)


class VrSettings(object):
    """
    Class containing VR settings pertaining to both the VR renderer
    and VR functionality in the simulator/of VR objects
    """
    def __init__(self):
        """
        Initializes VR settings.
        """
        # VR is enabled by default - can be set off on a case-by-case basis
        self.use_vr = True
        # Simulation is reset at start by default
        self.reset_sim = True
        # Fixed FPS is used by default
        self.use_fixed_fps = True

        mesh_renderer_folder = os.path.abspath(os.path.dirname(__file__))
        self.vr_config_path = os.path.join(mesh_renderer_folder, '..', '..', 'vr_config.json')
        self.load_vr_config()

    def load_vr_config(self):
        """
        Loads in VR config and sets all settings accordingly.
        """
        with open(self.vr_config_path) as f_config:
            config_str = ''.join(f_config.readlines())
            vr_config = json.loads(json_minify(config_str))
            
            shared_settings = vr_config['shared_settings']
            self.touchpad_movement = shared_settings['touchpad_movement']
            self.movement_controller = shared_settings['movement_controller']
            assert self.movement_controller in ['left', 'right']
            self.relative_movement_device = shared_settings['relative_movement_device']
            assert self.relative_movement_device in ['hmd', 'left_controller', 'right_controller']
            self.movement_speed = shared_settings['movement_speed']
            self.vr_fps = shared_settings['vr_fps']
            self.hud_width = shared_settings['hud_width']
            self.hud_pos = shared_settings['hud_pos']

            device_settings = vr_config['device_settings']
            curr_device_candidate = vr_config['current_device']
            if curr_device_candidate not in device_settings.keys():
                self.curr_device = 'OTHER_VR'
            else:
                self.curr_device = curr_device_candidate
            specific_device_settings = device_settings[self.curr_device]
            self.eye_tracking = specific_device_settings['eye_tracking']
            self.action_button_map = specific_device_settings['action_button_map']
            self.gen_button_action_map()

    def gen_button_action_map(self):
        """
        Generates a button_action_map, which is needed to convert from
        (button_idx, press_id) tuples back to actions.
        """
        self.button_action_map = {}
        for k, v in self.action_button_map.items():
            self.button_action_map[tuple(v)] = k

    def turn_off_vr_mode(self):
        """
        Turns off VR mode so the MeshRendererVR can be debugged.
        """
        self.use_vr = False

class MeshRendererVR(MeshRenderer):
    """
    MeshRendererVR is iGibson's VR rendering class. It handles rendering to the VR headset and provides
    a link to the underlying VRRendererContext, on which various functions can be called.
    """

    def __init__(self, rendering_settings=MeshRendererSettings(), vr_settings=VrSettings()):
        """
        :param rendering_settings: mesh renderer settings
        :param vr_settings: VR settings - see class definition above
        """
        self.vr_rendering_settings = rendering_settings
        self.vr_settings = vr_settings
        self.width = 1296
        self.height = 1440
        super().__init__(width=self.width, height=self.height, rendering_settings=self.vr_rendering_settings)

        # Rename self.r to self.vrsys
        self.vrsys = self.r
        if self.vr_settings.use_vr:
            # If eye tracking is requested but headset does not have eye tracking support, disable support
            if self.vr_settings.eye_tracking and not self.vrsys.hasEyeTrackingSupport():
                self.vr_settings.eye_tracking = False
            self.vrsys.initVR(self.vr_settings.eye_tracking)

        # Always turn MSAA off for VR
        self.msaa = False
        # The VrTextOverlay that serves as the VR HUD (heads-up-display)
        self.vr_hud = None

    def gen_vr_hud(self):
        """
        Generates VR HUD (heads-up-display).
        """
        # Create a unique overlay name based on current nanosecond
        uniq_name = 'overlay{}'.format(time.perf_counter())
        self.vr_hud = VrHUDOverlay(uniq_name, 
                                   self, 
                                   width=self.vr_settings.hud_width, 
                                   pos=self.vr_settings.hud_pos)
        self.vr_hud.set_overlay_show_state(True)

    def gen_static_overlay(self, image_fpath, width=1, pos=[0, 0, -1]):
        """
        Generates and returns an overlay containing a static image. This will display in addition to the HUD.
        """
        uniq_name = 'overlay{}'.format(time.perf_counter())
        static_overlay = VrStaticImageOverlay(uniq_name, 
                                    self, 
                                    image_fpath,
                                    width=width, 
                                    pos=pos)
        static_overlay.set_overlay_show_state(True)
        return static_overlay

    def update_vr_data(self):
        """
        Calls WaitGetPoses() to acquire pose data, and return 3ms before next vsync.
        """
        self.vrsys.updateVRData()

    def render(self):
        """
        Renders VR scenes.
        """
        if self.vr_settings.use_vr:
            left_proj, left_view, left_cam_pos, right_proj, right_view, right_cam_pos = self.vrsys.preRenderVR()

            # Render and submit left eye
            self.V = left_view
            self.P = left_proj
            # Set camera to be at the camera position of the VR eye
            self.camera = left_cam_pos
            # Set camera once for both VR eyes - use the right eye since this is what we save in data save and replay
            self.set_light_position_direction([right_cam_pos[0], right_cam_pos[1], 10], [right_cam_pos[0], right_cam_pos[1], 0])
            
            super().render(modes=('rgb'), return_buffer=False, render_shadow_pass=True)
            self.vrsys.postRenderVRForEye("left", self.color_tex_rgb)
            # Render and submit right eye
            self.V = right_view
            self.P = right_proj
            self.camera = right_cam_pos
            
            # We don't need to render the shadow pass a second time for the second eye
            # We also don't need to render the text pass a second time
            super().render(modes=('rgb'), return_buffer=False, render_shadow_pass=False, render_text_pass=False)
            self.vrsys.postRenderVRForEye("right", self.color_tex_rgb)

            # Update HUD so it renders in the HMD
            if self.vr_hud:
                self.vr_hud.refresh_text()
        else:
            super().render(modes=('rgb'), return_buffer=False, render_shadow_pass=True)

    def vr_compositor_update(self):
        """
        Submit data to VR compositor after rendering.
        """
        self.vrsys.postRenderVR(True)

    def release(self):
        """
        Releases Vr system and renderer.
        """
        super().release()
        if self.vr_settings.use_vr:
            self.vrsys.releaseVR()
