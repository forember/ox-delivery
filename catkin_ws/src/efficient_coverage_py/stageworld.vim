" Vim syntax file

syn region Comment start='#' end='$'
syn match Number /[-+]\?[0-9]\+\(\.[0-9]\+\)\?/

syn region String start='"' skip='\\"\|\\\\' end='"'

syn keyword Keyword model
syn keyword Keyword pose size origin update_interval
syn keyword Keyword color color_rgba bitmap ctrl
syn keyword Keyword fiducial_return fiducial_key obstacle_return blob_return
syn keyword Keyword ranger_return gripper_return
syn keyword Keyword gui_nose gui_grid gui_outline gui_move
syn keyword Keyword boundary mass map_resolution say alwayson
syn keyword Keyword stack_children

syn keyword Keyword actuator type axis min_position max_position max_speed
syn keyword Keyword blinkenlight size3 period dutycycle enabled
syn keyword Keyword blobfinder colors_count colors image range fov pan
syn keyword Keyword camera resolution pantilt watts
syn keyword Keyword fiducial range_min rang_max rang_max_id ignore_zloc
syn keyword Keyword gripper paddle_size paddle_state autosnatch

syn keyword Keyword position drive velocity localization localization_origin
syn keyword Keyword odom_error wheelbase velocity_bounds acceleration_bounds

syn keyword Keyword ranger sensor

syn keyword Keyword name interval_sim quit_time resolution show_clock
syn keyword Keyword show_clock_interval threads

syn keyword Keyword window center rotate scale pcam_loc pcam_angle
syn keyword Keyword show_data show_flags show_blocks show_clock show_footprints
syn keyword Keyword show_grid show_trailarrows show_trailrise show_trailfast
syn keyword Keyword show_occupancy show_tree pcam_on screenshots
syn keyword Keyword speedup

syn keyword Keyword define samples

syn match Identifier /[a-zA-Z_][a-zA-Z_0-9]*\((\|$\)\@=/
syn match Identifier /\(define\s\+\)\@<=[a-zA-Z_][a-zA-Z_0-9]*/
