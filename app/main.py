import harfang as hg
from math import pi
import math
from OrbitalCam import OrbitalController
from statistics import median


# helpers
def clamp(v, v_min, v_max):
	return max(v_min, min(v, v_max))

def rangeadjust_clamp(k, a, b, u, v):
	return rangeadjust(clamp(k, a, b), a, b, u, v)

def rangeadjust(k, a, b, u, v):
	return (k - a) / (b - a) * (v - u) + u

def lerp(k, a, b):
	return a + (b - a) * k

# initialize Harfang
hg.InputInit()
hg.WindowSystemInit()

keyboard = hg.Keyboard()
mouse = hg.Mouse()

res_x, res_y = 1920, 1080

# initialize lists and variables for toggle button (swipe style)
mousexlist = []
mouseylist = []
has_switched = False
dancing_mode = False

# Add these near the beginning of the file with other global variables
power_consumption = 0.0  # Total power consumed in joules
power_rate = {
    "idle": 0.5,         # Base power consumption when idle (watts)
    "movement": 2.0,     # Additional power when moving (watts per degree/sec)
    "holding": 0.8       # Power to maintain position against gravity (watts)
}
last_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Last recorded position for each joint

# Add these near the beginning of the file with other global variables
joint_temperatures = [25.0, 25.0, 25.0, 25.0, 25.0, 25.0]  # Initial temperature in Celsius
temperature_params = {
    "ambient": 22.0,       # Ambient temperature in Celsius
    "max_safe": 60.0,      # Maximum safe operating temperature
    "critical": 75.0,      # Critical temperature threshold
    "heating_rate": 0.15,  # How quickly motors heat up during movement
    "cooling_rate": 0.05,  # How quickly motors cool down when idle
    "power_factor": 0.3    # Temperature increase based on power consumption
}

win = hg.NewWindow("Harfang - Robot Simulator", res_x, res_y, 32)
hg.RenderInit(win)
hg.RenderReset(res_x, res_y, hg.RF_MSAA8X | hg.RF_FlipAfterRender |
			   hg.RF_FlushAfterRender | hg.RF_MaxAnisotropy)

hg.AddAssetsFolder("resources_compiled")

# AAA render params
aaa_config = hg.ForwardPipelineAAAConfig()
aaa_config.temporal_aa_weight = 0.01
aaa_config.sample_count = 1
aaa_config.motion_blur = 0.01
aaa_config.exposure = 1.925
aaa_config.gamma = 2.45
pipeline_aaa = hg.CreateForwardPipelineAAAFromAssets("core", aaa_config)
pipeline = hg.CreateForwardPipeline()
res = hg.PipelineResources()

# load scene
scene = hg.Scene()
hg.LoadSceneFromAssets("poppy.scn", scene, res, hg.GetForwardPipelineInfo())

# load texture for the quads
target_tex = hg.LoadTextureFromAssets("point.png", 0)[0]
texture_point = hg.MakeUniformSetTexture("s_texTexture", target_tex, 0)

target_tex = hg.LoadTextureFromAssets("Asset_1.png", 0)[0]
texture_asset1 = hg.MakeUniformSetTexture("s_texTexture", target_tex, 0)

target_tex = hg.LoadTextureFromAssets("Asset_2.png", 0)[0]
texture_asset2 = hg.MakeUniformSetTexture("s_texTexture", target_tex, 0)

target_tex = hg.LoadTextureFromAssets("switch-on.png", 0)[0]
texture_on = hg.MakeUniformSetTexture("s_texTexture", target_tex, 0)

target_tex = hg.LoadTextureFromAssets("switch-off.png", 0)[0]
texture_off = hg.MakeUniformSetTexture("s_texTexture", target_tex, 0)

# load shaders
render_state_quad = hg.ComputeRenderState(
	hg.BM_Alpha, hg.DT_Less, hg.FC_Disabled)
render_state_quad_occluded = hg.ComputeRenderState(
	hg.BM_Alpha, hg.DT_Less, hg.FC_Disabled)
render_state_line = hg.ComputeRenderState(
	hg.BM_Opaque, hg.DT_Less, hg.FC_Disabled)
shader_rotator = hg.LoadProgramFromAssets("shaders/rotator")
shader_for_plane = hg.LoadProgramFromAssets("shaders/texture")
shader_for_line = hg.LoadProgramFromAssets("shaders/pos_rgb")
shader_font = hg.LoadProgramFromAssets("core/shader/font")
vtx_layout = hg.VertexLayoutPosFloatTexCoord0UInt8()
vtx_line_layout = hg.VertexLayoutPosFloatColorUInt8()

# load font
font = hg.LoadFontFromAssets("Roboto-Regular.ttf", 36, 1024, 1,
							 "!\"#$%&'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~ ¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ")
text_render_state = hg.ComputeRenderState(
	hg.BM_Alpha, hg.DT_Always, hg.FC_Disabled, False)
font_color = hg.MakeUniformSetValue("u_color", hg.Vec4(1.0, 0.75, 0.2, 1.0))
font_color_white = hg.MakeUniformSetValue(
	"u_color", hg.Vec4(1.0, 1.0, 1.0, 1.0))
font_shadow = hg.MakeUniformSetValue("u_color", hg.Vec4(0.0, 0.0, 0.0, 1.0))

# array for each motor node and rotation axis choice
hg_motors = [
	{"n": scene.GetNode("bras_1"), "acc": 0, "v": 0, "axis": "Y", "lower_limit": -150, "upper_limit": 150, "new_position": 0.0, "quad_jauge_axis": hg.CreatePlaneModel(
		vtx_layout, 0.12, 0.12, 1, 1), "offset_slider": hg.Vec3(0, 0, 1), "offset_rotation": hg.Vec3(0, 0, 0), "axis_tex_big": True, "key_inc": hg.K_Q, "key_dec": hg.K_A},
	{"n": scene.GetNode("bras_2"), "acc": 0, "v": 0, "axis": "X", "lower_limit": -125, "upper_limit": 90, "new_position": 0.0, "quad_jauge_axis": hg.CreatePlaneModel(
		vtx_layout, 0.03, 0.03, 1, 1), "offset_slider": hg.Vec3(1, 0, 0), "offset_rotation": hg.Vec3(-1.57, 1.57, 0), "axis_tex_big": False, "key_inc": hg.K_W, "key_dec": hg.K_S},
	{"n": scene.GetNode("bras_3"), "acc": 0, "v": 0, "axis": "X", "lower_limit": -90, "upper_limit": 90, "new_position": 0.0, "quad_jauge_axis": hg.CreatePlaneModel(
		vtx_layout, 0.03, 0.03, 1, 1), "offset_slider": hg.Vec3(1, 0, 0), "offset_rotation": hg.Vec3(-1.57, 1.57, 0), "axis_tex_big": False, "key_inc": hg.K_E, "key_dec": hg.K_D},
	{"n": scene.GetNode("bras_4"), "acc": 0, "v": 0, "axis": "Z", "lower_limit": -150, "upper_limit": 150, "new_position": 0.0, "quad_jauge_axis": hg.CreatePlaneModel(
		vtx_layout, 0.06, 0.06, 1, 1), "offset_slider": hg.Vec3(0, 1, 0), "offset_rotation": hg.Vec3(0, 0, 0), "axis_tex_big": True, "key_inc": hg.K_R, "key_dec": hg.K_F},
	{"n": scene.GetNode("bras_5"), "acc": 0, "v": 0, "axis": "X", "lower_limit": -90, "upper_limit": 90, "new_position": 0.0, "quad_jauge_axis": hg.CreatePlaneModel(
		vtx_layout, 0.03, 0.03, 1, 1), "offset_slider": hg.Vec3(1, 0, 0), "offset_rotation": hg.Vec3(-1.57, 0, 1.57), "axis_tex_big": False, "key_inc": hg.K_T, "key_dec": hg.K_G},
	{"n": scene.GetNode("bras_6"), "acc": 0, "v": 0, "axis": "X", "lower_limit": -90, "upper_limit": 110, "new_position": 0.0, "quad_jauge_axis": hg.CreatePlaneModel(
		vtx_layout, 0.03, 0.03, 1, 1), "offset_slider": hg.Vec3(1, 0, 0), "offset_rotation": hg.Vec3(-1.57, 0, 1.57), "axis_tex_big": False, "key_inc": hg.K_Y, "key_dec": hg.K_H}
]

# if there is no camera add one
cam = None
for n in scene.GetAllNodes():
	if n.GetCamera().IsValid():
		cam = n
		break

if cam is None:
	cam = scene.CreateNode()
	cam.SetName("Camera")
	cam.SetTransform(scene.CreateTransform())
	cam.GetTransform().SetWorld(hg.TransformationMat4(
		hg.Vec3(0, 1000, 0), hg.Vec3(0, 0, 0)))
	cam.SetCamera(scene.CreateCamera(0.1, 10000))

scene.SetCurrentCamera(cam)
cam_tgt = hg.Vec3(0, 0.9, 0)
cam_pos = cam.GetTransform().GetPos()
cam_rot = cam.GetTransform().GetRot()

# load shader imgui
imgui_prg = hg.LoadProgramFromAssets("core/shader/imgui")
imgui_img_prg = hg.LoadProgramFromAssets("core/shader/imgui_image")

hg.ImGuiInit(10, imgui_prg, imgui_img_prg)

app_clock = 0

def toggle_button(label, value, x, y):
	global has_switched
	mat = hg.TransformationMat4(
		hg.Vec3(x, y, 1), hg.Vec3(0, 0, 0), hg.Vec3(1, 1, 1))
	pos = hg.GetT(mat)
	axis_x = hg.GetX(mat) * 56
	axis_y = hg.GetY(mat) * 24

	toggle_vtx = hg.Vertices(vtx_layout, 4)
	toggle_vtx.Begin(0).SetPos(
		pos - axis_x - axis_y).SetTexCoord0(hg.Vec2(0, 1)).End()
	toggle_vtx.Begin(1).SetPos(
		pos - axis_x + axis_y).SetTexCoord0(hg.Vec2(0, 0)).End()
	toggle_vtx.Begin(2).SetPos(
		pos + axis_x + axis_y).SetTexCoord0(hg.Vec2(1, 0)).End()
	toggle_vtx.Begin(3).SetPos(
		pos + axis_x - axis_y).SetTexCoord0(hg.Vec2(1, 1)).End()
	toggle_idx = [0, 3, 2, 0, 2, 1]

	hg.DrawTriangles(view_id, toggle_idx, toggle_vtx, shader_for_plane, [], [
		texture_on if value else texture_off], render_state_quad)

	if mouse.Down(hg.MB_0):
		mousexlist.append(mouse.X())
		mouseylist.append(mouse.Y())
	else:
		mousexlist.clear()
		mouseylist.clear()
		has_switched = False

	if len(mousexlist) > 20:
		mousexlist.pop(0)
	if len(mouseylist) > 20:
		mouseylist.pop(0)

	if len(mouseylist) > 0:
		mouse_x = median(mousexlist)
		mouse_y = median(mouseylist)
		if mouse_x > pos.x - axis_x.x and mouse_x < pos.x + axis_x.x and mouse_y > pos.y - axis_y.y and mouse_y < pos.y + axis_y.y and not has_switched:
			value = True if not value else False
			has_switched = True
			mousexlist.clear()
			mouseylist.clear()

	mat = hg.TranslationMat4(hg.Vec3(pos.x + axis_x.x + 10, y - 10, 1))
	hg.SetS(mat, hg.Vec3(1, -1, 1))
	hg.DrawText(view_id,
				font,
				label, shader_font, "u_tex", 0,
				mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color_white], [], text_render_state)
	return value

def get_v_from_dancing(id_robot):
	elapsed_time = app_clock * 0.5
	_amp = 1
	_freq = 0.5
	_offset = 0
	_phase = 0

	if id_robot == 0:
		_freq = 0.25
		_amp = 90.
	elif id_robot == 3:
		_freq = 0.25
		_amp = 90.
		_phase = 180.
	elif id_robot == 4:
		_freq = 0.8
		_amp = 30.
	elif id_robot == 5:
		_freq = 0.8
		_amp = 20.
		_phase = 180.
	else:
		_freq = 0.2
		_amp = 15.

	return _amp * math.sin(_freq * 2.0 * math.pi * elapsed_time + _phase * math.pi / 180.0) + _offset

# main loop
current_frame = 0
while not keyboard.Down(hg.K_Escape):
	keyboard.Update()
	mouse.Update()
	dt = hg.TickClock()

	app_clock += hg.time_to_sec_f(dt)

	render_was_reset, res_x, res_y = hg.RenderResetToWindow(
		win, res_x, res_y, hg.RF_VSync | hg.RF_MSAA4X | hg.RF_MaxAnisotropy)
	res_y = max(res_y, 16)

	world, cam_rot, cam_tgt, cam_pos = OrbitalController(
		keyboard, mouse, cam_pos, cam_rot, cam_tgt, dt, res_x, res_y)
	cam.GetTransform().SetWorld(world)

	scene.Update(dt)
	new_pass_views = hg.SceneForwardPipelinePassViewId()
	view_id = 0
	view_id, pass_ids = hg.SubmitSceneToPipeline(view_id, scene, hg.IntRect(
		0, 0, res_x, res_y), True, pipeline, res, pipeline_aaa, aaa_config, current_frame)

	view_state = scene.ComputeCurrentCameraViewState(
		hg.ComputeAspectRatioX(res_x, res_y))

	view_id_scene_opaque = hg.GetSceneForwardPipelinePassViewId(pass_ids, hg.SFPP_Opaque)
	view_id_scene_alpha = hg.GetSceneForwardPipelinePassViewId(pass_ids, hg.SFPP_Transparent)

	# Update and render each motor
	for id, m in enumerate(hg_motors):
		hg_m = hg_motors[id]

		# Handle keyboard input for manual control
		if not dancing_mode:
			if keyboard.Down(hg_m["key_inc"]):
				hg_m["v"] = clamp(hg_m["v"] + 60 * hg.time_to_sec_f(dt), 
								hg_m["lower_limit"], 
								hg_m["upper_limit"])
			if keyboard.Down(hg_m["key_dec"]):
				hg_m["v"] = clamp(hg_m["v"] - 60 * hg.time_to_sec_f(dt), 
								hg_m["lower_limit"], 
								hg_m["upper_limit"])
		else:
			# Dancing mode
			hg_m["v"] = get_v_from_dancing(id)

		# Update robot joint rotation
		rot = hg.Vec3(0, 0, 0)
		if hg_m["axis"] == "X":
			rot = hg.Vec3(-hg_m["v"]*pi/180.0, 0, 0)
		elif hg_m["axis"] == "Y":
			rot = hg.Vec3(0, -hg_m["v"]*pi/180.0, 0)
		elif hg_m["axis"] == "Z":
			rot = hg.Vec3(-1.57, 0, -hg_m["v"]*pi/180.0)

		hg_m["n"].GetTransform().SetRot(rot)

		# Draw joint visualization
		m_world = hg_m["n"].GetTransform().GetWorld()
		m_pos = hg.GetT(m_world)
		m_world_rot = hg.GetR(
			hg_m["n"].GetTransform().GetParent().GetTransform().GetWorld())
		m_world_scale = hg.GetS(m_world)

		if hg_m["offset_slider"].x == 1:
			m_pos += hg.GetX(m_world) * 0.03
		if hg_m["offset_slider"].y == 1:
			m_pos += hg.GetZ(m_world) * -0.015
		if hg_m["offset_slider"].z == 1:
			m_pos += hg.GetY(m_world) * 0.003

		m_world = hg.TransformationMat4(
			m_pos, m_world_rot, m_world_scale) * hg.RotationMat4(hg_m["offset_rotation"])
		hg_m["centroid_jauge_world"] = m_world

		progress = hg.MakeUniformSetValue("uProgress", hg.Vec4(
			rangeadjust_clamp(hg_m["v"], -180, 180, 0, 100)/100, 0, 0, 0))

		hg.DrawModel(view_id_scene_alpha, hg_m["quad_jauge_axis"], shader_rotator, [
			progress], [texture_asset1], m_world, render_state_quad_occluded)

	# Calculate power consumption for this frame
	frame_power = power_rate["idle"] * hg.time_to_sec_f(dt)  # Base idle power

	for id, m in enumerate(hg_motors):
		hg_m = hg_motors[id]
		
		# Calculate power from movement (proportional to speed)
		position_change = abs(hg_m["v"] - last_positions[id])
		movement_speed = position_change / hg.time_to_sec_f(dt) if hg.time_to_sec_f(dt) > 0 else 0
		movement_power = movement_speed * power_rate["movement"] * hg.time_to_sec_f(dt)
		
		# Calculate power from holding position against gravity
		# More power needed for horizontal positions than vertical
		angle_rad = abs(hg_m["v"] * pi / 180.0)
		gravity_factor = abs(math.sin(angle_rad)) if hg_m["axis"] != "Y" else abs(math.cos(angle_rad))
		holding_power = gravity_factor * power_rate["holding"] * hg.time_to_sec_f(dt)
		
		# Add this joint's power to the frame total
		frame_power += movement_power + holding_power
		
		# Update last position for next frame
		last_positions[id] = hg_m["v"]

	# Add frame power to total
	power_consumption += frame_power

	# Add this after the power consumption calculation but before drawing UI

	# Update joint temperatures based on movement and power consumption
	for id, m in enumerate(hg_motors):
		hg_m = hg_motors[id]
		current_temp = joint_temperatures[id]
		
		# Calculate temperature change based on movement and power consumption
		movement_speed = abs(hg_m["v"] - last_positions[id]) / hg.time_to_sec_f(dt) if hg.time_to_sec_f(dt) > 0 else 0
		
		# Calculate power for this specific joint
		angle_rad = abs(hg_m["v"] * pi / 180.0)
		gravity_factor = abs(math.sin(angle_rad)) if hg_m["axis"] != "Y" else abs(math.cos(angle_rad))
		holding_power = gravity_factor * power_rate["holding"] * hg.time_to_sec_f(dt)
		movement_power = movement_speed * power_rate["movement"] * hg.time_to_sec_f(dt)
		power_used = movement_power + holding_power
		
		 # Enhanced heating logic:
		# 1. Immediate temperature increase directly tied to movement
		# 2. Higher heating when near joint limits (motor strain)
		# 3. Additional heating from power consumption
		
		# Base temperature increase from movement
		temp_increase = 0.0000000001
		
		# Immediate response to movement (even small movements generate heat)
		if movement_speed > 0:
			# Base heating proportional to movement speed
			temp_increase = movement_speed * temperature_params["heating_rate"] * hg.time_to_sec_f(dt)
			
			# Extra heating when moving at high speeds
			if movement_speed > 40:  # Fast movement threshold
				temp_increase *= 1.5  # 50% more heat at high speeds
				
			# Extra heating when near limits (motor strain)
			limit_margin = 10  # degrees from limit where strain increases
			to_upper_limit = hg_m["upper_limit"] - hg_m["v"]
			to_lower_limit = hg_m["v"] - hg_m["lower_limit"]
			
			if to_upper_limit < limit_margin or to_lower_limit < limit_margin:
				# Calculate how close to the limit (0-1 range)
				limit_factor = 1.0 - min(to_upper_limit, to_lower_limit) / limit_margin
				if limit_factor > 0:  # Only apply if within the margin
					temp_increase *= (1.0 + limit_factor)  # Up to double heating at the limit
		
		# Additional heating from power consumption
		temp_increase += power_used * temperature_params["power_factor"]
		
		# Motors cool down toward ambient temperature
		cooling_factor = temperature_params["cooling_rate"] * hg.time_to_sec_f(dt)
		if current_temp > temperature_params["ambient"]:
			temp_decrease = cooling_factor * (current_temp - temperature_params["ambient"])
		else:
			temp_decrease = 0
		
		# Update temperature with all factors
		new_temp = current_temp + temp_increase - temp_decrease
		joint_temperatures[id] = new_temp

	# Draw UI
	hg.SetViewFrameBuffer(view_id, hg.InvalidFrameBufferHandle)
	hg.SetViewRect(view_id, 0, 0, res_x, res_y)
	hg.SetViewClear(view_id, 0, 0, 1.0, 0)

	vs = hg.ComputeOrthographicViewState(hg.TranslationMat4(
		hg.Vec3(res_x / 2, res_y / 2, 0)), res_y, 0.1, 100, hg.Vec2(res_x / res_y, 1))
	hg.SetViewTransform(view_id, vs.view, vs.proj)

	# Draw controls help text
	mat = hg.TranslationMat4(hg.Vec3(15, 150, 1))
	hg.SetS(mat, hg.Vec3(1, -1, 1))
	controls_text = "Controls:\nQ/A: Joint 1\nW/S: Joint 2\nE/D: Joint 3\nR/F: Joint 4\nT/G: Joint 5\nY/H: Joint 6"
	hg.DrawText(view_id,
				font,
				controls_text, shader_font, "u_tex", 0,
				mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color_white], [], text_render_state)

	# Draw angle display menu
	angle_display_x = res_x - 250
	angle_display_y = 100
	panel_width = 200
	panel_height = 180

	# Draw background panel for angle display
	panel_vtx = hg.Vertices(vtx_layout, 4)
	panel_vtx.Begin(0).SetPos(hg.Vec3(angle_display_x, angle_display_y, 1)).SetTexCoord0(hg.Vec2(0, 1)).End()
	panel_vtx.Begin(1).SetPos(hg.Vec3(angle_display_x, angle_display_y + panel_height, 1)).SetTexCoord0(hg.Vec2(0, 0)).End()
	panel_vtx.Begin(2).SetPos(hg.Vec3(angle_display_x + panel_width, angle_display_y + panel_height, 1)).SetTexCoord0(hg.Vec2(1, 0)).End()
	panel_vtx.Begin(3).SetPos(hg.Vec3(angle_display_x + panel_width, angle_display_y, 1)).SetTexCoord0(hg.Vec2(1, 1)).End()
	panel_idx = [0, 3, 2, 0, 2, 1]

	# Semi-transparent black background
	bg_color = hg.MakeUniformSetValue("uColor", hg.Vec4(0, 0, 0, 0.7))
	hg.DrawTriangles(view_id, panel_idx, panel_vtx, shader_for_plane, [bg_color], [], render_state_quad)

	# Draw title
	title_mat = hg.TranslationMat4(hg.Vec3(angle_display_x + 10, angle_display_y + 10, 1))
	hg.SetS(title_mat, hg.Vec3(1, -1, 1))
	hg.DrawText(view_id,
				font,
				"Joint Angles", shader_font, "u_tex", 0,
				title_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color], [], text_render_state)

	# Draw angle values for each motor
	for i, motor in enumerate(hg_motors):
		angle_mat = hg.TranslationMat4(hg.Vec3(angle_display_x + 10, angle_display_y + 40 + i * 22, 1))
		hg.SetS(angle_mat, hg.Vec3(1, -1, 1))
		
		# Format the angle value to 1 decimal place
		angle_text = f"Joint {i+1}: {motor['v']:.1f}°"
		
		hg.DrawText(view_id,
					font,
					angle_text, shader_font, "u_tex", 0,
					angle_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
					[font_color_white], [], text_render_state)

	# Add after the joint angles display

	# Draw power consumption panel
	power_display_x = res_x - 250
	power_display_y = angle_display_y + panel_height + 20
	power_panel_width = 200
	power_panel_height = 80

	# Draw background panel for power display
	power_panel_vtx = hg.Vertices(vtx_layout, 4)
	power_panel_vtx.Begin(0).SetPos(hg.Vec3(power_display_x, power_display_y, 1)).SetTexCoord0(hg.Vec2(0, 1)).End()
	power_panel_vtx.Begin(1).SetPos(hg.Vec3(power_display_x, power_display_y + power_panel_height, 1)).SetTexCoord0(hg.Vec2(0, 0)).End()
	power_panel_vtx.Begin(2).SetPos(hg.Vec3(power_display_x + power_panel_width, power_display_y + power_panel_height, 1)).SetTexCoord0(hg.Vec2(1, 0)).End()
	power_panel_vtx.Begin(3).SetPos(hg.Vec3(power_display_x + power_panel_width, power_display_y, 1)).SetTexCoord0(hg.Vec2(1, 1)).End()
	power_panel_idx = [0, 3, 2, 0, 2, 1]

	# Semi-transparent black background
	hg.DrawTriangles(view_id, power_panel_idx, power_panel_vtx, shader_for_plane, [bg_color], [], render_state_quad)

	# Draw title
	power_title_mat = hg.TranslationMat4(hg.Vec3(power_display_x + 10, power_display_y + 10, 1))
	hg.SetS(power_title_mat, hg.Vec3(1, -1, 1))
	hg.DrawText(view_id,
				font,
				"Power Consumption", shader_font, "u_tex", 0,
				power_title_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color], [], text_render_state)

	# Display total power consumption
	power_text_mat = hg.TranslationMat4(hg.Vec3(power_display_x + 10, power_display_y + 40, 1))
	hg.SetS(power_text_mat, hg.Vec3(1, -1, 1))

	# Format power values - show joules for small values, kJ for larger values
	power_text = f"Total: {power_consumption:.2f} J" if power_consumption < 1000 else f"Total: {power_consumption/1000:.2f} kJ"

	hg.DrawText(view_id,
				font,
				power_text, shader_font, "u_tex", 0,
				power_text_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color_white], [], text_render_state)

	# Display current power rate
	rate_text_mat = hg.TranslationMat4(hg.Vec3(power_display_x + 10, power_display_y + 60, 1))
	hg.SetS(rate_text_mat, hg.Vec3(1, -1, 1))
	current_rate = frame_power / hg.time_to_sec_f(dt) if hg.time_to_sec_f(dt) > 0 else 0
	rate_text = f"Current: {current_rate:.2f} W"

	hg.DrawText(view_id,
				font,
				rate_text, shader_font, "u_tex", 0,
				rate_text_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color_white], [], text_render_state)

	# Add after the power consumption panel, before the dancing mode toggle

	# Draw temperature monitoring panel
	temp_display_x = res_x - 250
	temp_display_y = power_display_y + power_panel_height + 20
	temp_panel_width = 200
	temp_panel_height = 180

	# Draw background panel for temperature display
	temp_panel_vtx = hg.Vertices(vtx_layout, 4)
	temp_panel_vtx.Begin(0).SetPos(hg.Vec3(temp_display_x, temp_display_y, 1)).SetTexCoord0(hg.Vec2(0, 1)).End()
	temp_panel_vtx.Begin(1).SetPos(hg.Vec3(temp_display_x, temp_display_y + temp_panel_height, 1)).SetTexCoord0(hg.Vec2(0, 0)).End()
	temp_panel_vtx.Begin(2).SetPos(hg.Vec3(temp_display_x + temp_panel_width, temp_display_y + temp_panel_height, 1)).SetTexCoord0(hg.Vec2(1, 0)).End()
	temp_panel_vtx.Begin(3).SetPos(hg.Vec3(temp_display_x + temp_panel_width, temp_display_y, 1)).SetTexCoord0(hg.Vec2(1, 1)).End()
	temp_panel_idx = [0, 3, 2, 0, 2, 1]

	# Semi-transparent black background
	hg.DrawTriangles(view_id, temp_panel_idx, temp_panel_vtx, shader_for_plane, [bg_color], [], render_state_quad)

	# Draw title
	temp_title_mat = hg.TranslationMat4(hg.Vec3(temp_display_x + 10, temp_display_y + 10, 1))
	hg.SetS(temp_title_mat, hg.Vec3(1, -1, 1))
	hg.DrawText(view_id,
				font,
				"Motor Temperatures", shader_font, "u_tex", 0,
				temp_title_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
				[font_color], [], text_render_state)

	# Replace the temperature bar visualization with a simpler text-based display

	# Draw temperature values for each motor
	for i, temp in enumerate(joint_temperatures):
		# Text position
		temp_mat = hg.TranslationMat4(hg.Vec3(temp_display_x + 10, temp_display_y + 40 + i * 22, 1))
		hg.SetS(temp_mat, hg.Vec3(1, -1, 1))
		
		# Determine temperature status
		status = ""
		if temp >= temperature_params["critical"]:
			status = " [CRITICAL]" 
			temp_color = hg.MakeUniformSetValue("u_color", hg.Vec4(1.0, 0.0, 0.0, 1.0))  # Red for critical
		elif temp >= temperature_params["max_safe"]:
			status = " [WARNING]"
			temp_color = hg.MakeUniformSetValue("u_color", hg.Vec4(1.0, 0.5, 0.0, 1.0))  # Orange for warning
		else:
			temp_color = hg.MakeUniformSetValue("u_color", hg.Vec4(0.0, 1.0, 0.0, 1.0))  # Green for normal
		
		# Format temperature value with status indicator
		temp_text = f"Joint {i+1}: {temp:.1f}°C{status}"
		
		# Draw temperature text
		hg.DrawText(view_id,
					font,
					temp_text, shader_font, "u_tex", 0,
					temp_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
					[temp_color], [], text_render_state)
		
		# Add temperature threshold information below the values
		if i == len(joint_temperatures) - 1:
			threshold_mat = hg.TranslationMat4(hg.Vec3(temp_display_x + 10, temp_display_y + 50 + len(joint_temperatures) * 22, 1))
			hg.SetS(threshold_mat, hg.Vec3(1, -1, 1))
			threshold_text = f"Safe: <{temperature_params['max_safe']:.1f}°C | Critical: >{temperature_params['critical']:.1f}°C"
			
			hg.DrawText(view_id,
						font,
						threshold_text, shader_font, "u_tex", 0,
						threshold_mat, hg.Vec3(0, 0, 0), hg.DTHA_Left, hg.DTVA_Top,
						[font_color_white], [], text_render_state)

	# Toggle dancing mode
	dancing_mode = toggle_button(
		"Dancing Mode ON" if dancing_mode else "Dancing Mode OFF", dancing_mode, 100, res_y - 80)

	view_id += 1
	current_frame = hg.Frame()
	hg.UpdateWindow(win)

hg.RenderShutdown()
hg.DestroyWindow(win)