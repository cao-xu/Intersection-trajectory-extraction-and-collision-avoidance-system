# SET THE PATH AND CONFIG
SOURCE_FOLDER="G:/trajectory-extractor/mytest/test19" # p1 # C:/Users/Fred/PycharmProjects/trajectory-extractor/mytest/test17
VIDEO_NAME="test19.mp4" # p2
VIDEO_PATH=${SOURCE_FOLDER}/${VIDEO_NAME}
NAME="test19" # p3
DELTA_MS=100
LOCATION_NAME="shanghai"
DATE="20201210"
START_TIME="1210"

CAMERA_STREET="area1_street_cfg.yml"
CAMERA_SAT="sat_cfg.yml"
CAMERA_SAT_IMG="sat.png"
HD_MAP="area1_street_hd_map.csv"

# SET DETECTION AD IGNORE ZONES
DET_ZONE_IM_VEHICLES=${NAME}"_detection_zone_im.yml"
DET_ZONE_FNED_VEHICLES=${NAME}"_detection_zone.yml"
IGNORE_AREA_VEHICLES=""

DET_ZONE_IM_PEDESTRIANS=${NAME}"_detection_zone_im.yml"
DET_ZONE_FNED_PEDESTRIANS=${NAME}"_detection_zone.yml"
IGNORE_AREA_PEDESTRIANS=""

# SET CROP VALUES FOR DETECTION HERE IF NEEDED
CROP_X1=180
CROP_Y1=120
CROP_X2=1250
CROP_Y2=720

IMG_OUTPUT_FOLDER=${SOURCE_FOLDER}
TIME_MAX_S="20"
SKIP="2"
IMG_DIR=${SOURCE_FOLDER}"/img"
OUTPUT_DIR=${SOURCE_FOLDER}"/output"

DET_DIR=${OUTPUT_DIR}"/det/csv"

MODE_VEHICLES="vehicles"
DYNAMIC_MODEL_VEHICLES="BM2"
LABEL_REPLACE_VEHICLES="car"
OUTPUT_VEHICLES_DIR=${OUTPUT_DIR}/${MODE_VEHICLES}
DET_ASSO_VEHICLES_DIR=${OUTPUT_VEHICLES_DIR}"/det_association/csv"
TRAJ_VEHICLES_DIR=${OUTPUT_VEHICLES_DIR}"/traj/csv"
TRACK_MERGE_VEHICLES=${OUTPUT_VEHICLES_DIR}"/det_association/"${NAME}"_tracks_merge.csv"
TRAJ_VEHICLES=${TRAJ_VEHICLES_DIR}/${NAME}"_traj.csv"
TRAJ_INSPECT_VEHICLES_DIR=${OUTPUT_VEHICLES_DIR}"/traj_inspect/csv"
TRAJ_INSPECT_VEHICLES=${TRAJ_INSPECT_VEHICLES_DIR}/${NAME}"_traj.csv"
TRAJ_INSPECT_VEHICLES_TIME=${TRAJ_INSPECT_VEHICLES_DIR}/${NAME}"_time_traj.csv"

TRAJ_NOT_MERGED_CSV=${TRAJ_INSPECT_VEHICLES_DIR}/${NAME}"_traj_not_merged.csv"
SHRINK_ZONE=1
MIN_LENGTH=6

MODE_PEDESTRIANS="pedestrians"
DYNAMIC_MODEL_PEDESTRIANS="CV"
LABEL_REPLACE_PEDESTRIANS="person"
OUTPUT_PEDESTRIANS_DIR=${OUTPUT_DIR}/${MODE_PEDESTRIANS}
DET_ASSO_PEDESTRIANS_DIR=${OUTPUT_PEDESTRIANS_DIR}"/det_association/csv"
TRAJ_PEDESTRIANS_DIR=${OUTPUT_PEDESTRIANS_DIR}"/traj/csv"
TRACK_MERGE_PEDESTRIANS=${OUTPUT_PEDESTRIANS_DIR}"/det_association/"${NAME}"_tracks_merge.csv"
TRAJ_PEDESTRIANS=${TRAJ_PEDESTRIANS_DIR}/${NAME}"_traj.csv"
TRAJ_INSPECT_PEDESTRIANS_DIR=${OUTPUT_PEDESTRIANS_DIR}"/traj_inspect/csv"
TRAJ_INSPECT_PEDESTRIANS=${TRAJ_INSPECT_PEDESTRIANS_DIR}/${NAME}"_traj.csv"

#############################################################
# 从视频中提取图片
#############################################################
# 参数根据实际视频具体再定
#python traj_ext/object_det/run_saveimages.py ${VIDEO_PATH}\
#        -o ${IMG_OUTPUT_FOLDER}\
#        -t ${TIME_MAX_S}\
#        --skip ${SKIP}

#read -p "任意键继续..."

CALIB_POINTS_PATH=${SOURCE_FOLDER}/"street_camera_calib_manual_latlon.csv"
IMAGE_PATH=${SOURCE_FOLDER}/"area1_street.jpg"
#IMAGE_PATH=${SOURCE_FOLDER}/"sat.png"
#############################################################
#标定街景视图相机
############################################################
#python traj_ext/camera_calib/run_calib_manual.py\
#        -calib_point ${CALIB_POINTS_PATH}\
#        -image ${IMAGE_PATH}\
#        -output_folder ${SOURCE_FOLDER}

#read -p "任意键继续..."

CAM_MODEL_PATH=${SOURCE_FOLDER}/"area1_street_cfg.yml"
ORIGIN_LAT="37.858056"
ORIGIN_LON="112.529669"
##########################################################
#画底图
##########################################################
#python traj_ext/hd_map/run_generate_HD_map.py\
#        -image ${IMAGE_PATH}\
#        -camera ${CAM_MODEL_PATH}\
#        -origin_lat ${ORIGIN_LAT}\
#        -origin_lon ${ORIGIN_LON}\
#        -output_folder ${SOURCE_FOLDER}

#read -p "任意键继续..."

CAM_MODEL_STREET_PATH=${SOURCE_FOLDER}/"area1_street_cfg.yml"
CAM_MODEL_SAT_PATH=${SOURCE_FOLDER}/"sat_cfg.yml"
IMAGE_SAT_PATH=${SOURCE_FOLDER}/"sat.png"
ZONE_OUTPUT_NAME=${NAME}
###########################################################
#画检测区域
###########################################################
#python traj_ext/camera_calib/run_detection_zone.py\
#        -camera_street ${CAM_MODEL_STREET_PATH}\
#        -image_street ${IMAGE_PATH}\
#        -camera_sat ${CAM_MODEL_SAT_PATH}\
#        -image_sat ${IMAGE_SAT_PATH}\
#        -output_name ${ZONE_OUTPUT_NAME}

#read -p "任意键继续..."

####################################################################
# OBJECT DETECTION
####################################################################

#python traj_ext/object_det/mask_rcnn/run_detections_csv.py\
#        -image_dir ${IMG_DIR}\
#        -output_dir ${OUTPUT_DIR}\
#        -crop_x1y1x2y2 ${CROP_X1} ${CROP_Y1} ${CROP_X2} ${CROP_Y2}\
#        -no_save_images

#####################################################################
## VEHICLES
#####################################################################
#
# Det association
#python traj_ext/det_association/run_det_association.py\
#            -image_dir ${IMG_DIR}\
#            -output_dir ${OUTPUT_VEHICLES_DIR}\
#            -det_dir ${DET_DIR}\
#            -ignore_detection_area ${SOURCE_FOLDER}/${IGNORE_AREA_VEHICLES}\
#            -det_zone_im ${SOURCE_FOLDER}/${DET_ZONE_IM_VEHICLES}\
#            -mode ${MODE_VEHICLES}\
#            -no_save_images

#read -p "任意键继续..."
##
### Process
#python traj_ext/postprocess_track/run_postprocess.py\
#            -image_dir ${IMG_DIR}\
#            -output_dir ${OUTPUT_VEHICLES_DIR}\
#            -det_dir ${DET_DIR}\
#            -det_asso_dir ${DET_ASSO_VEHICLES_DIR}\
#            -track_merge ${TRACK_MERGE_VEHICLES}\
#            -camera_street ${SOURCE_FOLDER}/${CAMERA_STREET}\
#            -camera_sat  ${SOURCE_FOLDER}/${CAMERA_SAT}\
#            -camera_sat_img ${SOURCE_FOLDER}/${CAMERA_SAT_IMG}\
#            -det_zone_fned ${SOURCE_FOLDER}/${DET_ZONE_FNED_VEHICLES}\
#            -delta_ms ${DELTA_MS}\
#            -location_name ${LOCATION_NAME}\
#            -dynamic_model ${DYNAMIC_MODEL_VEHICLES}\
#            -date ${DATE}\
#            -start_time ${START_TIME}\
#            -no_save_images

#read -p "任意键继续..."
###
### inspect
#python traj_ext/visualization/run_inspect_traj.py\
#            -traj ${TRAJ_VEHICLES}\
#            -image_dir ${IMG_DIR}\
#            -det_dir ${DET_DIR}\
#            -det_asso_dir ${DET_ASSO_VEHICLES_DIR}\
#            -track_merge ${TRACK_MERGE_VEHICLES}\
#            -camera_street ${SOURCE_FOLDER}/${CAMERA_STREET}\
#            -camera_sat  ${SOURCE_FOLDER}/${CAMERA_SAT}\
#            -camera_sat_img ${SOURCE_FOLDER}/${CAMERA_SAT_IMG}\
#            -det_zone_fned ${SOURCE_FOLDER}/${DET_ZONE_FNED_VEHICLES}\
#            -label_replace ${LABEL_REPLACE_VEHICLES}\
#            -output_dir ${OUTPUT_VEHICLES_DIR}\
#            -hd_map ${SOURCE_FOLDER}/${HD_MAP}\
#            -delta_ms ${DELTA_MS}\
#            -location_name ${LOCATION_NAME}\
#            -date ${DATE}\
#            -start_time ${START_TIME}\
#            -export

##################################################################
# 冲突检测
##################################################################
#python conflict_detection/vehicle_conflict_detect.py \
#            -traj ${TRAJ_VEHICLES}\
#            -output_dir ${OUTPUT_DIR}

#################################################################
# 打开可视化窗口
#################################################################
#python controller.py


#################################################################
# 制作视频
#################################################################
MAKE_VIDEO_FOLDER=${OUTPUT_DIR}/"visualizer/make_video"
ORIGIN_IMG_PATH=${OUTPUT_DIR}/"visualizer/img_concat"
RESULT_VIDEO_PATH=${OUTPUT_DIR}/"visualizer"
# 实现：拷贝原视频图片到make_video文件夹、重命名所有照片、并制作视频
#python make_video.py \
#             -video_img_path ${MAKE_VIDEO_FOLDER} \
#             -origin_img_path ${ORIGIN_IMG_PATH} \
#             -result_video_path ${RESULT_VIDEO_PATH}

###################################################################
# VISUALIZATION
###################################################################

#python traj_ext/visualization/run_visualizer.py\
#            -traj ${TRAJ_INSPECT_VEHICLES}\
#            -image_dir ${IMG_DIR}\
#            -camera_street ${SOURCE_FOLDER}/${CAMERA_STREET}\
#            -camera_sat  ${SOURCE_FOLDER}/${CAMERA_SAT}\
#            -camera_sat_img ${SOURCE_FOLDER}/${CAMERA_SAT_IMG}\
#            -det_zone_fned ${SOURCE_FOLDER}/${DET_ZONE_FNED_VEHICLES}\
#            -hd_map ${SOURCE_FOLDER}/${HD_MAP}\
#            -output_dir ${OUTPUT_DIR}\
