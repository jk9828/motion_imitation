"""Run from motion_imitation/retarget_motion to find data correctly."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

import time

import tensorflow.compat.v1 as tf
import numpy as np

from motion_imitation.utilities import pose3d
from pybullet_utils import transformations
import pybullet
import pybullet_data as pd
from motion_imitation.utilities import motion_util

# import retarget_config_a1 as config
#import retarget_config_laikago as config
import retarget_config_marubot as config
# import retarget_config_vision60 as config

POS_SIZE = 3
ROT_SIZE = 4
DEFAULT_ROT = np.array([1, 0, 0, 1])
FORWARD_DIR = np.array([1, 0, 0])

GROUND_URDF_FILENAME = "plane_implicit.urdf"

# reference motion
FRAME_DURATION = 0.01667
REF_COORD_ROT = transformations.quaternion_from_euler(0, 0, 0)
REF_POS_OFFSET = np.array([1, 0, 0])
REF_ROOT_ROT = transformations.quaternion_from_euler(0, 0, 0)

REF_PELVIS_JOINT_ID = 0
REF_NECK_JOINT_ID = 0
REF_SHOUL_JOINT_IDS = [1, 6]
REF_CLE_JOINT_IDS = [11, 16]
#REF_HIP_JOINT_IDS = [6, 16, 11, 20]
REF_HIP_JOINT_IDS = [ 2, 12, 7, 17]
#REF_TOE_JOINT_IDS = [10, 19, 15, 23]
REF_TOE_JOINT_IDS = [5, 15, 10, 20]

REF_SHOUL_KNEE_JOINT_ID = [4, 13, 9 ,18]

mocap_motions = [
  ["Human motion", "data/bone_animation_data.txt", 1,200],
]

  
  # right turn0
  #JOINT_POS_FILENAME = "data/dog_walk09_joint_pos.txt"
  #FRAME_START = 2404
  #FRAME_END = 2450
  

def build_markers(num_markers): # func buil_markers(num_markers) : 상체/하체/발의 마커를 생성 및 색을 입히는 함수 -> 마커를 리턴

  marker_radius = 0.02

  markers = []
  for i in range(num_markers):
    if (i == REF_NECK_JOINT_ID) or (i == REF_PELVIS_JOINT_ID)\
        or (i in REF_HIP_JOINT_IDS):
      col = [0, 0, 1, 1]
    elif (i in REF_TOE_JOINT_IDS):
      col = [1, 0, 0, 1]
    else:
      col = [0, 1, 0, 1]

    virtual_shape_id = pybullet.createVisualShape(shapeType=pybullet.GEOM_SPHERE,
                                                  radius=marker_radius,
                                                  rgbaColor=col)
    body_id =  pybullet.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=virtual_shape_id,
                                  basePosition=[0,0,0],
                                  useMaximalCoordinates=True)
    markers.append(body_id)

  return markers

"""
def build_markers(num_markers): # * func buil_markers(num_markers) : 상체/하체/발의 마커를 생성 및 색을 입히는 함수 -> 마커를 리턴

  marker_radius = 0.02

  check = 20
  markers = []
  for i in range(num_markers):
    if i == check:
      col = [0, 0, 1, 1]
    else:
      col = [0, 1, 0, 1]

    virtual_shape_id = pybullet.createVisualShape(shapeType=pybullet.GEOM_SPHERE,                                  
                                                  radius=marker_radius,
                                                  rgbaColor=col)
    body_id =  pybullet.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=-1,
                                  baseVisualShapeIndex=virtual_shape_id,
                                  basePosition=[0,0,0],
                                  useMaximalCoordinates=True)
    markers.append(body_id)

  return markers
"""


def get_joint_limits(robot): # * func get_joint_limits(robot) : 로봇의 각 조인트의 가동범위를 받아옴 -> joint_limit_low, joint_limit_high 리턴
  num_joints = pybullet.getNumJoints(robot)
  joint_limit_low = []
  joint_limit_high = []

  for i in range(num_joints):
    joint_info = pybullet.getJointInfo(robot, i)
    joint_type = joint_info[2]

    if (joint_type == pybullet.JOINT_PRISMATIC or joint_type == pybullet.JOINT_REVOLUTE):
      joint_limit_low.append(joint_info[8])
      joint_limit_high.append(joint_info[9])

  return joint_limit_low, joint_limit_high

def get_root_pos(pose): # * func get_root_pos(pose) : 로봇의 위치 정보 리턴
  return pose[0:POS_SIZE]

def get_root_rot(pose): # * func get_root_rot(pose) : 로봇의 회전 정보 리턴
  return pose[POS_SIZE:(POS_SIZE + ROT_SIZE)]

def get_joint_pose(pose): # * func get_joint_pose(pose) : 로봇의 조인트 정보 리턴
  return pose[(POS_SIZE + ROT_SIZE):]

def set_root_pos(root_pos, pose): # * pose(포즈 정보를 모두 담은 리스트) 에 root_pos 정보 넣기
  pose[0:POS_SIZE] = root_pos
  return

def set_root_rot(root_rot, pose):
  pose[POS_SIZE:(POS_SIZE + ROT_SIZE)] = root_rot # pose(포즈 정보를 모두 담은 리스트) 에 root_rot 정보 넣기
  return

def set_joint_pose(joint_pose, pose): # * pose(포즈 정보를 모두 담은 리스트) 에 joint_pose 정보 넣기 // 참고 : pose 는 pos/rot/jointpos 순으로 정보를 가진다.
  pose[(POS_SIZE + ROT_SIZE):] = joint_pose
  return

def set_pose(robot, pose):
  num_joints = pybullet.getNumJoints(robot) 
  root_pos = get_root_pos(pose)
  root_rot = get_root_rot(pose)
  pybullet.resetBasePositionAndOrientation(robot, root_pos, root_rot) # 파이불릿의 이 함수를 적용하면 로봇의 base_position이 변경되는듯

  for j in range(num_joints):
    j_info = pybullet.getJointInfo(robot, j) # 조인트의 마찰/최대 속도/가동 범위 등의 정보를 저장
    j_state = pybullet.getJointStateMultiDof(robot, j) # 조인트의 현재 각도 정보 속도 정보를 가진다.

    j_pose_idx = j_info[3] # the first position index in the positional state variables for this body 절대 좌표계 위치가 아닐까 싶음
    j_pose_size = len(j_state[0]) # 조인트의 포지션의 데이터수 아마 조인트 개수 * 4 ?
    j_vel_size = len(j_state[1]) # 조인트의 속도의 데이터수  아마 조인트 개수 * 3 ?

    if (j_pose_size > 0):
      j_pose = pose[j_pose_idx:(j_pose_idx + j_pose_size)] # 데이터개수로 딱 포지션에 관련된 데이터만 갈무리
      j_vel = np.zeros(j_vel_size) # 데이터개수로 딱 속도에 관련된 데이터만 갈무리
      pybullet.resetJointStateMultiDof(robot, j, j_pose, j_vel) # 로봇의 위치 및 속도 조정 실제로 움직임

  return

def set_maker_pos(marker_pos, marker_ids): # * 걍 마커 찍는 함수 크게 안중요
  num_markers = len(marker_ids)
  assert(num_markers == marker_pos.shape[0]) 

  for i in range(num_markers):
    curr_id = marker_ids[i]
    curr_pos = marker_pos[i]

    pybullet.resetBasePositionAndOrientation(curr_id, curr_pos, DEFAULT_ROT)

  return



def process_ref_joint_pos_data(joint_pos): # * 모션 데이터를 파이불릿에 넣을 수 있는 데이터로 바꿈
  proc_pos = joint_pos.copy()
  num_pos = joint_pos.shape[0]

  for i in range(num_pos):
    curr_pos = proc_pos[i]
    curr_pos = pose3d.QuaternionRotatePoint(curr_pos, REF_COORD_ROT)
    curr_pos = pose3d.QuaternionRotatePoint(curr_pos, REF_ROOT_ROT)
    curr_pos = curr_pos * config.REF_POS_SCALE + REF_POS_OFFSET
    proc_pos[i] = curr_pos

  return proc_pos


"""
def retarget_root_pose(ref_joint_pos): # * 루트의 포즈 및 회전각을 리턴 하는 코드
  pelvis_pos = ref_joint_pos[REF_PELVIS_JOINT_ID] # 골반 조인트 포지션 저장
  neck_pos = ref_joint_pos[REF_NECK_JOINT_ID] # 목쪽 조인트 포지션 저장

  left_shoulder_pos = ref_joint_pos[REF_HIP_JOINT_IDS[0]] 
  right_shoulder_pos = ref_joint_pos[REF_HIP_JOINT_IDS[2]]
  left_hip_pos = ref_joint_pos[REF_HIP_JOINT_IDS[1]]
  right_hip_pos = ref_joint_pos[REF_HIP_JOINT_IDS[3]]


  delta_hip_shoulder_R = right_soulder_pos - right_hip_pos
  delta_hip_shoulder_L = left_soulder_pos - left_hip_pos

  up_dir = (delta_hip_shoulder_R + delta_hip_shoulder_L)/2
  up_dir = up_dir/np.linalg.norm(up_dir)

  delta_shoulder = left_shoulder_pos - right_shoulder_pos
  delta_hip = left_hip_pos - right_hip_pos

  dir_shoulder = delta_shoulder / np.linalg.norm(delta_shoulder) # 오른쪽이 base 왼쪽이 end 인 방향 벡터 - shoulder
  dir_hip = delta_hip / np.linalg.norm(delta_hip) # 오른쪽 base 왼쪽이 end 인 방향 벡터 - hip

  left_dir = (delta_hip + delta_shoulder)/2
  
  forward_dir = npcross(left_dir, up_dir)
  forward_dir = forward_dir / np.linalg.norm(left_dir)


  rot_mat = np.array([[forward_dir[0], left_dir[0], up_dir[0], 0],
                      [forward_dir[1], left_dir[1], up_dir[1], 0],
                      [forward_dir[2], left_dir[2], up_dir[2], 0],
                      [0, 0, 0, 1]]) # 로테이션 매트릭스 결정

  root_pos = 0.5 * (pelvis_pos + neck_pos) # 포즈는 그냥 쉽게 중간으로 감
  #root_pos = 0.25 * (left_shoulder_pos + right_shoulder_pos + left_hip_pos + right_hip_pos) 다른 방법도 있긴함
  root_rot = transformations.quaternion_from_matrix(rot_mat)
  root_rot = transformations.quaternion_multiply(root_rot, config.INIT_ROT)
  root_rot = root_rot / np.linalg.norm(root_rot)

  return root_pos, root_rot # 루트 포즈, 회전각 리턴
"""


def retarget_root_pose(ref_joint_pos): # * 루트의 포즈 및 회전각을 리턴 하는 코드
  pelvis_pos = ref_joint_pos[REF_PELVIS_JOINT_ID] # 골반 조인트 포지션 저장
  neck_pos = ref_joint_pos[REF_NECK_JOINT_ID] # 목쪽 조인트 포지션 저장

  cle_pos_1 = ref_joint_pos[REF_CLE_JOINT_IDS[0]]
  cle_pos_2 = ref_joint_pos[REF_CLE_JOINT_IDS[1]]

  left_shoulder_pos = ref_joint_pos[REF_HIP_JOINT_IDS[0]] 
  right_shoulder_pos = ref_joint_pos[REF_HIP_JOINT_IDS[2]]
  left_hip_pos = ref_joint_pos[REF_HIP_JOINT_IDS[1]]
  right_hip_pos = ref_joint_pos[REF_HIP_JOINT_IDS[3]]

  middle_hip = (right_hip_pos + left_hip_pos)/2


  up_dir = pelvis_pos - middle_hip
  up_dir = up_dir/np.linalg.norm(up_dir)

  delta_shoulder = left_shoulder_pos - right_shoulder_pos
  delta_hip = left_hip_pos - right_hip_pos


  left_dir = (delta_hip + delta_shoulder)/2
  left_dir = left_dir/np.linalg.norm(left_dir)
  
  forward_dir = np.cross(left_dir, up_dir)
  forward_dir = forward_dir / np.linalg.norm(forward_dir)


  rot_mat = np.array([[forward_dir[0], left_dir[0], up_dir[0], 0],
                      [forward_dir[1], left_dir[1], up_dir[1], 0],
                      [forward_dir[2], left_dir[2], up_dir[2], 0],
                      [0, 0, 0, 1]]) # 로테이션 매트릭스 결정

  

  root_pos = 0.5 * (pelvis_pos + (cle_pos_1 + cle_pos_2)/2) # 포즈는 그냥 쉽게 중간으로 감
  #root_pos = 0.25 * (left_shoulder_pos + right_shoulder_pos + left_hip_pos + right_hip_pos) 다른 방법도 있긴함
  root_rot = transformations.quaternion_from_matrix(rot_mat)
  root_rot = transformations.quaternion_multiply(root_rot, config.INIT_ROT)
  root_rot = root_rot / np.linalg.norm(root_rot)

  return root_pos, root_rot # 루트 포즈, 회전각 리턴


def retarget_pose(robot, default_pose, ref_joint_pos): # * 전체 포즈 리타게팅 함수
  joint_lim_low, joint_lim_high = get_joint_limits(robot) # URDF 로부터 joint limit 리턴

  root_pos, root_rot = retarget_root_pose(ref_joint_pos) # 루트 먼저 리타겟
  root_pos += config.SIM_ROOT_OFFSET

  pybullet.resetBasePositionAndOrientation(robot, root_pos, root_rot)

  inv_init_rot = transformations.quaternion_inverse(config.INIT_ROT) 
  heading_rot = motion_util.calc_heading_rot(transformations.quaternion_multiply(root_rot, inv_init_rot)) # 처음 회전을 초기화 하는 거인듯? 퀀터니안을 잘이해 못하겠음


  tar_tip_pos = []
  tar_toe_pos = []
  tar_shoul_knee_pos = [] 
  for i in range(len(REF_TOE_JOINT_IDS)): 
    ref_toe_id = REF_TOE_JOINT_IDS[i] 
    ref_hip_id = REF_HIP_JOINT_IDS[i] # 레퍼런스의 TOE/HIP 아이디 저장
    ref_shoul_knee_id = REF_SHOUL_KNEE_JOINT_ID[i]

    sim_hip_id = config.SIM_HIP_JOINT_IDS[i] 
    toe_offset_local = config.SIM_TOE_OFFSET_LOCAL[i] # 시뮬레이션의 TOE/HIP 아이디 저장

    ref_toe_pos = ref_joint_pos[ref_toe_id]
    ref_hip_pos = ref_joint_pos[ref_hip_id] # 대응 포즈 저장
    ref_shoul_knee_pos = ref_joint_pos[ref_shoul_knee_id]

    hip_link_state = pybullet.getLinkState(robot, sim_hip_id, computeForwardKinematics=True) # 링크의 연결지지점의 포지션을 받아옴 computeForwardKinematics=True 옵션에의해 글로벌 카르테시안 오리엔테이션 으로줌
    sim_hip_pos = np.array(hip_link_state[4]) # 링크 시작점 포지션

    toe_offset_world = pose3d.QuaternionRotatePoint(toe_offset_local, heading_rot) # local 의 로테이션 정도와 local에 대한 정보로 global 계산

    ref_hip_toe_delta = ref_toe_pos - ref_hip_pos 
    sim_tar_toe_pos = sim_hip_pos + ref_hip_toe_delta
    sim_tar_toe_pos[2] = ref_toe_pos[2] # z 는 그냥 같게 설정
    sim_tar_toe_pos += toe_offset_world


    ref_hip_shoul_knee_delta = ref_shoul_knee_pos - ref_hip_pos 
    sim_tar_shoul_knee_pos = sim_hip_pos + ref_hip_shoul_knee_delta
    #sim_tar_shoul_knee_pos[2] = ref_shoul_knee_pos[2] # z 는 그냥 같게 설정
    sim_tar_shoul_knee_pos += toe_offset_world

    tar_tip_pos.append(sim_tar_shoul_knee_pos)
    tar_tip_pos.append(sim_tar_toe_pos)

  for i in range(len(REF_SHOUL_JOINT_IDS)): 

    ref_shoul_id = REF_SHOUL_JOINT_IDS[i]

    sim_hip_id = config.SIM_HIP_JOINT_IDS[i] 
    toe_offset_local = config.SIM_TOE_OFFSET_LOCAL[i] # 시뮬레이션의 TOE/HIP 아이디 저장

    ref_toe_pos = ref_joint_pos[ref_toe_id]
    ref_hip_pos = ref_joint_pos[ref_hip_id] # 대응 포즈 저장
    ref_shoul_pos = ref_joint_pos[ref_shoul_id]

    hip_link_state = pybullet.getLinkState(robot, sim_hip_id, computeForwardKinematics=True) # 링크의 연결지지점의 포지션을 받아옴 computeForwardKinematics=True 옵션에의해 글로벌 카르테시안 오리엔테이션 으로줌
    sim_hip_pos = np.array(hip_link_state[4]) # 링크 시작점 포지션

    toe_offset_world = pose3d.QuaternionRotatePoint(toe_offset_local, heading_rot) # local 의 로테이션 정도와 local에 대한 정보로 global 계산

    ref_hip_shoul_delta = ref_shoul_pos - ref_hip_pos
    sim_tar_shoul_pos = sim_hip_pos + ref_hip_shoul_delta

    tar_tip_pos.append(sim_tar_shoul_pos)

  joint_pose = pybullet.calculateInverseKinematics2(robot, config.SIM_TIP_JOINT_IDS,
                                                    tar_tip_pos,
                                                    jointDamping=config.JOINT_DAMPING,
                                                    lowerLimits=joint_lim_low,
                                                    upperLimits=joint_lim_high,
                                                    restPoses=default_pose) # 파이불릿 내장 함수로 전부 해결 


  joint_pose = np.array(joint_pose)
  pose = np.concatenate([root_pos, root_rot, joint_pose])

  return pose

def update_camera(robot): # * 카메라 위치/각도 업데이트
  base_pos = np.array(pybullet.getBasePositionAndOrientation(robot)[0])
  [yaw, pitch, dist] = pybullet.getDebugVisualizerCamera()[8:11] # 카메라의 요 피치 디스턴스 받기
  pybullet.resetDebugVisualizerCamera(dist, yaw, pitch, base_pos) # 위의 것과 더해서 base pose 만 수정
  return

def load_ref_data(JOINT_POS_FILENAME, FRAME_START, FRAME_END): # 레퍼런스 모션 데이터를 받아옴
  joint_pos_data = np.loadtxt(JOINT_POS_FILENAME, delimiter=",")

  start_frame = 0 if (FRAME_START is None) else FRAME_START
  end_frame = joint_pos_data.shape[0] if (FRAME_END is None) else FRAME_END
  joint_pos_data = joint_pos_data[start_frame:end_frame]

  return joint_pos_data

def retarget_motion(robot, joint_pos_data): # 프레임별 리타겟 포지션을 저장하고 리턴
  num_frames = joint_pos_data.shape[0]

  for f in range(num_frames):
    ref_joint_pos = joint_pos_data[f] 
    ref_joint_pos = np.reshape(ref_joint_pos, [-1, POS_SIZE])
    ref_joint_pos = process_ref_joint_pos_data(ref_joint_pos) # 모션데이터를 파이불릿에 사용할 수 있는 데이터로 바꿈

    curr_pose = retarget_pose(robot, config.DEFAULT_JOINT_POSE, ref_joint_pos) # 팔다리 조인트 리타게팅
    set_pose(robot, curr_pose)

    if f == 0:
      pose_size = curr_pose.shape[-1] 
      new_frames = np.zeros([num_frames, pose_size])

    new_frames[f] = curr_pose

  new_frames[:, 0:2] -= new_frames[0, 0:2]

  return new_frames 

def output_motion(frames, out_filename):
  with open(out_filename, "w") as f:
    f.write("{\n")
    f.write("\"LoopMode\": \"Wrap\",\n")
    f.write("\"FrameDuration\": " + str(FRAME_DURATION) + ",\n")
    f.write("\"EnableCycleOffsetPosition\": true,\n")
    f.write("\"EnableCycleOffsetRotation\": true,\n")
    f.write("\n")

    f.write("\"Frames\":\n")

    f.write("[")
    for i in range(frames.shape[0]):
      curr_frame = frames[i]

      if i != 0:
        f.write(",")
      f.write("\n  [")

      for j in range(frames.shape[1]):
        curr_val = curr_frame[j]
        if j != 0:
          f.write(", ")
        f.write("%.5f" % curr_val)

      f.write("]")

    f.write("\n]")
    f.write("\n}")

  return

def main(argv):
  
  p = pybullet
  p.connect(p.GUI, options="--width=1920 --height=1080 --mp4=\"test.mp4\" --mp4fps=60")
  p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)

  pybullet.setAdditionalSearchPath(pd.getDataPath())
  

  while True:
    
    for mocap_motion in mocap_motions: # mocap_motion은 레퍼런스 데이터들, 걷기, 뛰기, 등등
      pybullet.resetSimulation()
      pybullet.setGravity(0, 0, 0)
    
      ground = pybullet.loadURDF(GROUND_URDF_FILENAME) # 환경정보
      robot = pybullet.loadURDF(config.URDF_FILENAME, config.INIT_POS, config.INIT_ROT) # 로봇
      # Set robot to default pose to bias knees in the right direction.
      set_pose(robot, np.concatenate([config.INIT_POS, config.INIT_ROT, config.DEFAULT_JOINT_POSE])) # 로봇의 init 포즈를 정의, 근데 왜하는지 모르겠음ㄴ

      p.removeAllUserDebugItems()
      print("mocap_name=", mocap_motion[0])
      
      joint_pos_data = load_ref_data(mocap_motion[1],mocap_motion[2],mocap_motion[3])
    
      num_markers = joint_pos_data.shape[-1] // POS_SIZE
      marker_ids = build_markers(num_markers)
    
      retarget_frames = retarget_motion(robot, joint_pos_data)
      output_motion(retarget_frames, f"{mocap_motion[0]}.txt")
    
      f = 0
      num_frames = joint_pos_data.shape[0]
    
      for repeat in range (num_frames):
        time_start = time.time()
    
        f_idx = f % num_frames
        print("Frame {:d}".format(f_idx))
    
        ref_joint_pos = joint_pos_data[f_idx]
        ref_joint_pos = np.reshape(ref_joint_pos, [-1, POS_SIZE])
        ref_joint_pos = process_ref_joint_pos_data(ref_joint_pos)
    
        pose = retarget_frames[f_idx]
    
        set_pose(robot, pose)
        set_maker_pos(ref_joint_pos, marker_ids)
    
        update_camera(robot)
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
        f += 1
    
        time_end = time.time()
        sleep_dur = FRAME_DURATION - (time_end - time_start)
        sleep_dur = max(0, sleep_dur)
    
        time.sleep(sleep_dur)
        #time.sleep(0.5) # jp hack
      for m in marker_ids:
        p.removeBody(m)
      marker_ids = []

  pybullet.disconnect()

  return


if __name__ == "__main__":
  tf.app.run(main)

