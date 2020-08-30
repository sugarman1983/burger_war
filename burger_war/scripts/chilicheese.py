#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

import tf

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2

PI = math.pi
# 8x8  [rad]
TARGET_TH = (
    (-PI/4, -PI/4, -PI/2, -PI/2, -PI*3/4, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/3, -PI/2, -PI*3/5, -PI*3/4, -PI*3/4, -PI*3/4),
    (-PI/4, -PI/4, -PI/6,     0,   -PI/2, -PI*3/4,     -PI,  PI*3/4),
    (-PI/4, -PI/5,     0,     0,      PI,  PI*6/10,  PI*3/4,    PI/2),
    (    0,     0,  PI/2,  PI/2,      PI,  PI*3/4,  PI*3/4,    PI/2),
    (    0,  PI/4,  PI/3,  PI/2,  PI*5/6,  PI*3/4,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/3,  PI*5/6,    PI/2,  PI*3/4,  PI*3/4),
    ( PI/4,  PI/4,  PI/4,  PI/4,    PI/3,    PI/2,  PI*3/4,  PI*3/4),
)   #向くべき方向を決めるためのマトリックス
WIDTH = 1.2 * (2 **0.5) # [m]　2**0.5は√2を表す。すなわちフィールドの対角線の半分の長さ

# respect is_point_enemy freom team rabbit
# https://github.com/TeamRabbit/burger_war
class EnemyDetector:
    '''
    Lidarのセンサ値から簡易的に敵を探す。
    obstacle detector などを使ったほうがROSらしいがそれは参加者に任せます。
    いろいろ見た感じ Team Rabit の実装は綺麗でした。
    方針
    実測のLidarセンサ値 と マップと自己位置から期待されるLidarセンサ値 を比較
    ズレている部分が敵と判断する。
    この判断は自己位置が更新された次のライダーのコールバックで行う。（フラグで管理）
    0.7m 以上遠いところは無視する。少々のズレは許容する。
    '''
    #OK
    def __init__(self):
        self.max_distance = 1.0     #検知距離の最大値
        self.thresh_corner = 0.25   #コーナー障害物の大きさ(当たり判定)
        self.thresh_center = 0.35   #センター障害物の大きさ(当たり判定)

        self.pose_x = 0             #自機のX座標
        self.pose_y = 0             #自機のY座標
        self.th = 0                 #自機の角度
    

    #OK
    def findEnemy(self, scan, pose_x, pose_y, th):
        '''
        input scan. list of lider range, robot locate(pose_x, pose_y, th)
        return is_near_enemy(BOOL), enemy_direction[rad](float)
        '''
        if not len(scan) == 360:    #エラーチェック
            return False
        
        # update pose
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.th = th

        # drop too big and small value ex) 0.0 , 2.0 
        near_scan = [x if self.max_distance > x > 0.1 else 0.0 for x in scan]
            #scanの値がmax_distanceと0.1の間であればその値を、そうでなければ0.0をnear_scanに追加
            #   near_scan = []
            #       for x in scan:
            #           if self.max_distance > x > 0.1:
            #               near_scan.append(x)
            #           else:
            #               near_scan.append(0.0)

        enemy_scan = [1 if self.is_point_emnemy(x,i) else 0 for i,x in  enumerate(near_scan)]
            #near_scanのインデックス番号iと要素xを取得し、is_point_enemyへ代入し、返り値があれば1、なければ０をenemy_scanに追加
            #   enemy_scan = []
            #       for i,x in enumerate(near_scan):
            #           if self.is_point_enemy(x,i):
            #               enemy_scan.append(1)
            #           else:
            #               enemy_scan.appen(0)

        is_near_enemy = sum(enemy_scan) > 5  # if less than 5 points, maybe noise
        if is_near_enemy:
            idx_l = [i for i, x in enumerate(enemy_scan) if x == 1]
                #enemy_scanのインデックス番号iと要素xを取得し、xが1ならば、idx_lにインデックス番号iを追加
                #   idx_l = []
                #   for i,x in enumerate(enemy_scan):
                #       if x == 1:
                #           idx_l.append(i)
            idx = idx_l[len(idx_l)/2]   #idx_lの要素数を取得し2で割る。つまり、敵の中心となるインデックス番号を求めidxへ挿入
            enemy_direction = idx / 360.0 * 2*PI    #敵の中心の方向を求め、enemy_directionへ代入
            enemy_dist = near_scan[idx] #敵の距離を求め、enemy_distへ代入
        else:
            enemy_direction = None  #敵の検知なし
            enemy_dist = None       #敵の検知なし

        print("Enemy: {}, Direction: {}".format(is_near_enemy, enemy_direction))
        print("enemy points {}".format(sum(enemy_scan)))
        return is_near_enemy, enemy_direction, enemy_dist
        

    def is_point_emnemy(self, dist, ang_deg):   #遮蔽物の座標値を確認し、フィールド内にいる、かつ、障害物でなければTrue
        if dist == 0:       #エラーチェック
            return False

        ang_rad = ang_deg /360. * 2 * PI    #角度変換deg→rad
        point_x = self.pose_x + dist * math.cos(self.th + ang_rad)  #敵のX座標
        point_y = self.pose_y + dist * math.sin(self.th + ang_rad)  #敵のY座標

        #フィールド内かチェック　対角線の半分1.70-機体の大きさ0.17=1.53　敵のX座標とY座標の絶対値の合計が1.53を超えないかチェック
        if   point_y > (-point_x + 1.53):
            return False
        elif point_y < (-point_x - 1.53):
            return False
        elif point_y > ( point_x + 1.53):
            return False
        elif point_y < ( point_x - 1.53):
            return False

        #フィールド内の物体でないかチェック
        len_p1 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y - 0.53), 2)) #敵のコーナー障害物1の中心からの距離
        len_p2 = math.sqrt(pow((point_x - 0.53), 2) + pow((point_y + 0.53), 2)) #敵のコーナー障害物2の中心からの距離
        len_p3 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y - 0.53), 2)) #敵のコーナー障害物3の中心からの距離
        len_p4 = math.sqrt(pow((point_x + 0.53), 2) + pow((point_y + 0.53), 2)) #敵のコーナー障害物4の中心からの距離
        len_p5 = math.sqrt(pow(point_x         , 2) + pow(point_y         , 2)) #敵のセンター障害物の中心からの距離

        if len_p1 < self.thresh_corner or len_p2 < self.thresh_corner or len_p3 < self.thresh_corner or len_p4 < self.thresh_corner or len_p5 < self.thresh_center:
            return False    #障害物の大きさより小さければエラー
        else:
            #print(point_x, point_y, self.pose_x, self.pose_y, self.th, dist, ang_deg, ang_rad)
            #print(len_p1, len_p2, len_p3, len_p4, len_p5)
            return True

# End Respect

class ChiliCheeseBurger():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


        # bot name 
        self.name = 'bot_name'
        # robot state 'inner' or 'outer'
        self.state = 'inner' 
        # robot wheel rot 
        self.wheel_rot_r = 0    #右車輪の回転変数の宣言
        self.wheel_rot_l = 0    #左車輪の回転変数の宣言
        self.pose_x = 0         #自機のX座標変数の宣言
        self.pose_y = 0         #自機のY座標変数の宣言
        self.th = 0             #自機の角度変数の宣言

        self.k = 0.5
        self.near_wall_range = 0.2  # [m]

        # speed [m/s]
        self.speed = 0.07       #走行速度設定

        self.is_near_wall = 0   #壁面検知の初期化
        
        # lidar scan
        self.scan = []          #LiDARの数値

        self.pose_twist = Twist()

        ## publisher
        #self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback) #自己位置の取得
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)                    #lidarのデータに基づく行動

        #直進
        #self.pose_twist = Twist()
        self.pose_twist.linear.x = self.speed; self.pose_twist.linear.y = 0.; self.pose_twist.linear.z = 0.
        self.pose_twist.angular.x = 0.; self.pose_twist.angular.y = 0.; self.pose_twist.angular.z = 0.

        #敵検知時の変数初期化および動作初期化
        self.is_near_enemy = False
        self.enemy_direction = None
        self.enemy_dist = None
        self.near_enemy_twist = Twist()
        self.near_enemy_twist.linear.x = self.speed; self.near_enemy_twist.linear.y = 0.; self.near_enemy_twist.linear.z = 0.
        self.near_enemy_twist.angular.x = 0.; self.near_enemy_twist.angular.y = 0.; self.near_enemy_twist.angular.z = 0.

        #自己位置の初期化をFalseに切り替え
        self.is_initialized_pose = False
        #敵検知クラスを実行
        self.enemy_detector = EnemyDetector()


    #OK
    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        self.pose_x = data.pose.pose.position.x #自機のx座標の取得
        self.pose_y = data.pose.pose.position.y #自機のy座標の取得
        quaternion = data.pose.pose.orientation #自機のクォータニオン取得
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))    #クォータニオン→オイラー

        self.th = rpy[2]    #自機の角度取得
        th_xy = self.calcTargetTheta(self.pose_x,self.pose_y)   #現在地より向くべき方向を決定
        
        self.updatePoseTwist(self.th, th_xy)                    #遮蔽物の情報より回転角を決定
        self.is_initialized_pose = True                         #is_initialized_poseをTrueに変更

    #OK
    def updatePoseTwist(self, th, th_xy):
        # update pose twist
        th_diff = th_xy - th    #自機の角度と向くべき方向の差を求める
        while not PI >= th_diff >= -PI: #差が-PI〜PIの範囲に収まるよう修正
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        
        delta_th = self.calcDeltaTheta(th_diff) #周囲の遮蔽物情報を元に向くべき方向に対する調整角を決定
        new_twist_ang_z = max(-0.3, min((th_diff + delta_th) * self.k , 0.3))   #回転角を-0.3〜0.3の間にする
        
        self.pose_twist.angular.z = new_twist_ang_z #回転角を代入
        self.pose_twist.linear.x = self.speed       #速度を代入(デフォルト値)
        #print("th: {}, th_xy: {}, delta_th: {}, new_twist_ang_z: {}".format(th, th_xy, delta_th, new_twist_ang_z))

    #OK
    def calcTargetTheta(self, pose_x, pose_y):
        x = self.poseToindex(pose_x)    #x座標をフィールドを8分割したインデックス番号で置き換える
        y = self.poseToindex(pose_y)    #y座標をフィールドを8分割したインデックス番号で置き換える
        th = TARGET_TH[x][y]            #xとyのインデックス番号より向くべき方向のマトリックスで方向を決定
        #print("POSE pose_x: {}, pose_y: {}. INDEX x:{}, y:{}".format(pose_x, pose_y, x, y))
        return th

    #OK?
    def calcDeltaTheta(self, th_diff):
        if not self.scan:   #scanデータがあるか確認
            return 0.
        R0_idx = self.radToidx(th_diff - PI/8)  #向かうべき方向の右22.5°
        R1_idx = self.radToidx(th_diff - PI/4)  #向かうべき方向の右45°
        L0_idx = self.radToidx(th_diff + PI/8)  #向かうべき方向の左22.5°
        L1_idx = self.radToidx(th_diff + PI/4)  #向かうべき方向の左45°
        R0_range = 99. if self.scan[R0_idx] < 0.1 else self.scan[R0_idx]    #向かうべき方向の右22.5°の遮蔽物との距離が0.1以下なら99、それ以上なら距離の値
        R1_range = 99. if self.scan[R1_idx] < 0.1 else self.scan[R1_idx]    #向かうべき方向の右45°の遮蔽物との距離が0.1以下なら99、それ以上なら距離の値
        L0_range = 99. if self.scan[L0_idx] < 0.1 else self.scan[L0_idx]    #向かうべき方向の左22.5°の遮蔽物との距離が0.1以下なら99、それ以上なら距離の値
        L1_range = 99. if self.scan[L1_idx] < 0.1 else self.scan[L1_idx]    #向かうべき方向の左45°の遮蔽物との距離が0.1以下なら99、それ以上なら距離の値

        #print("Ranges R0: {}, R1: {}, L0: {}, L1: {}".format(R0_range, R1_range, L0_range, L1_range))
        if R0_range < 0.3 and L0_range > 0.3:   #向かうべき方向の右22.5°の遮蔽物との距離が0.3以下かつ左22.5°の遮蔽物との距離が0.3以上ならPI/4=左45°
            return PI/4
        elif R0_range > 0.3 and L0_range < 0.3: #向かうべき方向の右22.5°の遮蔽物との距離が0.3以上かつ左22.5°の遮蔽物との距離が0.3以下なら-PI/4=右45°
            return -PI/4
        elif R1_range < 0.2 and L1_range > 0.2: #向かうべき方向の右45°の遮蔽物との距離が0.2以下かつ左45°の遮蔽物との距離が0.2以上ならPI/8=左22.5°
            return PI/8
        elif R1_range > 0.2 and L1_range < 0.2: #向かうべき方向の右45°の遮蔽物との距離が0.2以上かつ左45°の遮蔽物との距離が0.2以下ならPI/8=左22.5°
            return -PI/8
        else:
            return 0.   #それ以外の場合0を返す
    
    #OK
    def radToidx(self, rad):
        deg = int(rad / (2*PI) * 360)   #rad→deg
        while not 360 > deg >= 0:   #degが0〜360の範囲に収まるよう修正
            if deg > 0:
                deg -= 360
            elif deg < 0:
                deg += 360
        return deg

    #OK
    def poseToindex(self, pose):    #フィールドを8分割し、自機がどのエリアにいるかをインデックス番号化
        i = 7 - int((pose + WIDTH) / (2 * WIDTH) * 8)   #インデックス番号i=-1~7
        i = max(0, min(7, i))                           #インデックス番号i=0~7(-1~0の間は0になる)？
        return i

    #OK
    def lidarCallback(self, data):
        '''
        lidar scan use for bumper , and find enemy
        controll speed.x
        '''
        scan = data.ranges  #lidarデータをscanに代入
        self.scan = scan
        self.is_near_wall = self.isNearWall(scan)   #壁に接近していないかチェック
        
        # enemy detection
        if self.is_initialized_pose:
            self.is_near_enemy, self.enemy_direction, self.enemy_dist = self.enemy_detector.findEnemy(scan, self.pose_x, self.pose_y, self.th)
            #敵フラグ、敵の方向、敵の距離を代入
        if self.is_near_enemy:
            self.updateNearEnemyTwist() #敵が近くにいる場合の動作を開始する

    #OK
    def updateNearEnemyTwist(self): #敵が近くにいる場合の動作内容
        # update pose twist
        th_diff = self.enemy_direction
        while not PI >= th_diff >= -PI: #敵の方向値が-π〜πの間でない場合、収まるように変換
            if th_diff > 0:
                th_diff -= 2*PI
            elif th_diff < 0:
                th_diff += 2*PI
        new_twist_ang_z = max(-0.3, min((th_diff) * self.k , 0.3))  #敵の方向へ回転する角度を-0.3〜0.3の範囲に制限

        if self.enemy_dist > 0.36:
            speed = -self.speed  #敵が0.36以上離れていれば前進
        else:
            speed = -self.speed #敵が0.36より近ければ後退
        #print("enemy_dist {}".format(self.enemy_dist))
        
        self.near_enemy_twist.angular.z = new_twist_ang_z
        self.near_enemy_twist.linear.x = speed
        #print("Update near Enemy Twist")

    #OK
    def isNearWall(self, scan):
        if not len(scan) == 360:    #エラーチェック
            return False
        forward_scan = scan[:15] + scan[-15:]   #正面のscan値
        # drop too small value ex) 0.0
        forward_scan = [x for x in forward_scan if x > 0.1]
            #   forward_scanの値が0.1以下を落とす
            #   for x in forward_scan:
            #       if x > 0.1:
            #           forward_scan.append(x)
        backward_scan = scan[165:195]   #背面のscan値
        # drop too small value ex) 0.0
        backward_scan = [x for x in backward_scan if x > 0.1]

        if min(forward_scan) < 0.2: #forward_scanの最小値が0.2以下なら壁ありと判定
            return 1

        if min(backward_scan) < 0.2: #forward_scanの最小値が0.2以下なら壁ありと判定
            return -1
        
        return False



    def setGoal(self,x,y,yaw):
        # Waits until the action server has started up and started listening for goals.
        self.client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move x meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = x
        # Move y meters forward along the y axis of the "map" coordinate frame
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        # Sends the goal to the action server.
        self.client.send_goal(goal)
        # Waits for the server to finish performing the action.
        wait = self.client.wait_for_result()
        # If the result doesn't arrive, assume the Server is not available
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return self.client.get_result()

    def cruise(self):
        self.area = 0
        if (self.pose_y <= -self.pose_x and self.pose_y >= self.pose_x):
            self.area = 1
        elif (self.pose_y <= self.pose_x and self.pose_y <= -self.pose_x):
            self.area = 2
        elif (self.pose_y >= -self.pose_x and self.pose_y <= self.pose_x): 
            self.area = 3
        else:   #elif ((self.pose_y >= self.pose_x) and (self.pose_y > -self.pose_x)):
            self.area = 4

        self.target_location_ccw = ((-0.5,0.15,-3.1415*3/4),(-0.1,-0.4,-3.1415/4),(0.5,-0.15,3.1415/4),(0.1,0.4,3.1415*3/4))
        self.target_location_cw  = ((0.5,0.15,-3.1415/4),(-0.1,0.4,3.1415/4),(-0.5,-0.15,3.1415*3/4),(0.1,-0.4,-3.1415*3/4))

        if self.cruise_dir == 1:
            if self.area == 1:
                self.setGoal(-0.9,0.4,3.1415/4)
                self.setGoal(-0.9,0.4,-3.1415/2)
                self.setGoal(-0.9,-0.4,-3.1415/4)
                self.setGoal(-0.9,-0.4,3.1415/4)
                self.setGoal(-0.6,-0.1,0)

                self.setGoal(self.target_location_ccw[1][0],self.target_location_ccw[1][1],self.target_location_ccw[1][2])

            elif self.area == 2:
                  
                self.setGoal(0,-0.6,0)
                self.setGoal(0,-0.6,3.1415/2)
                self.setGoal(0,-0.6,3.1415)
                self.setGoal(0,-0.6,3.1415/4)

                self.setGoal(self.target_location_ccw[2][0],self.target_location_ccw[2][1],self.target_location_ccw[2][2])

            elif self.area == 3:
                self.setGoal(0.9,-0.4,-3.1415*3/4)
                self.setGoal(0.9,-0.4,3.1415/2)
                self.setGoal(0.9,0.4,3.1415*3/4)
                self.setGoal(0.6,0.1,3.1415)

                self.setGoal(self.target_location_ccw[3][0],self.target_location_ccw[3][1],self.target_location_ccw[3][2])

            elif self.area == 4:
                self.setGoal(0,0.6,3.1415)
                self.setGoal(0,0.6,-3.1415/2)
                self.setGoal(0,0.6,0)
                self.setGoal(0,0.6,-3.1415*3/4)

                self.setGoal(self.target_location_ccw[0][0],self.target_location_ccw[0][1],self.target_location_ccw[0][2])

        elif self.cruise_dir == -1:
            if self.area == 1:
                self.setGoal(-0.9,-0.4,-3.1415/4)
                self.setGoal(-0.9,-0.4,3.1415/2)
                self.setGoal(-0.9,0.4,3.1415/4)
                self.setGoal(-0.9,0.4,-3.1415/4)
                self.setGoal(-0.6,0.1,0)

                self.setGoal(self.target_location_cw[1][0],self.target_location_cw[1][1],self.target_location_cw[1][2])

            elif self.area == 2:
                  
                self.setGoal(0,-0.6,3.1415)
                self.setGoal(0,-0.6,3.1415/2)
                self.setGoal(0,-0.6,0)
                self.setGoal(0,-0.6,3.1415*3/4)

                self.setGoal(self.target_location_cw[2][0],self.target_location_cw[2][1],self.target_location_cw[2][2])

            elif self.area == 3:
                self.setGoal(0.9,0.4,3.1415*3/4)
                self.setGoal(0.9,0.4,-3.1415/2)
                self.setGoal(0.9,-0.4,-3.1415*3/4)
                self.setGoal(0.6,-0.1,3.1415)

                self.setGoal(self.target_location_cw[3][0],self.target_location_cw[3][1],self.target_location_cw[3][2])

            elif self.area == 4:
                self.setGoal(0,0.6,0)
                self.setGoal(0,0.6,-3.1415/2)
                self.setGoal(0,0.6,-3.1415)
                self.setGoal(0,0.6,-3.1415/4)

                self.setGoal(self.target_location_cw[0][0],self.target_location_cw[0][1],self.target_location_cw[0][2])

        #else:
            #return 1

 
    def strategy(self):
        r = rospy.Rate(10) # change speed 10fps

        self.cruise_dir = 1

        while not rospy.is_shutdown():

            twist = Twist()
            
            if not self.is_near_enemy:
                #self.cruise_dir = -self.cruise_dir
                self.cruise()
                
            else:      
                print("Detect Enemy!")
                self.client.cancel_goal
                twist.linear.x = self.near_enemy_twist.linear.x
                twist.angular.z = self.near_enemy_twist.angular.z

                if self.cruise_dir == -1:
                    if (self.pose_y <= -self.pose_x and self.pose_y >= self.pose_x):
                        self.setGoal(self.target_location_ccw[2][0],self.target_location_ccw[2][1],self.target_location_ccw[2][2])
                    elif (self.pose_y <= self.pose_x and self.pose_y <= -self.pose_x):
                        self.setGoal(self.target_location_ccw[3][0],self.target_location_ccw[3][1],self.target_location_ccw[3][2])
                    elif (self.pose_y >= -self.pose_x and self.pose_y <= self.pose_x): 
                        self.setGoal(self.target_location_ccw[0][0],self.target_location_ccw[0][1],self.target_location_ccw[0][2])
                    else:   #elif ((self.pose_y >= self.pose_x) and (self.pose_y > -self.pose_x)):
                        self.setGoal(self.target_location_ccw[1][0],self.target_location_ccw[1][1],self.target_location_ccw[1][2])
                elif self.cruise_dir == 1:
                    if (self.pose_y <= -self.pose_x and self.pose_y >= self.pose_x):
                        self.setGoal(self.target_location_cw[0][0],self.target_location_cw[0][1],self.target_location_cw[0][2])
                    elif (self.pose_y <= self.pose_x and self.pose_y <= -self.pose_x):
                        self.setGoal(self.target_location_cw[1][0],self.target_location_cw[1][1],self.target_location_cw[1][2])
                    elif (self.pose_y >= -self.pose_x and self.pose_y <= self.pose_x): 
                        self.setGoal(self.target_location_cw[2][0],self.target_location_cw[2][1],self.target_location_cw[2][2])
                    else:   #elif ((self.pose_y >= self.pose_x) and (self.pose_y > -self.pose_x)):
                        self.setGoal(self.target_location_cw[3][0],self.target_location_cw[3][1],self.target_location_cw[3][2])

                self.cruise_dir = -self.cruise_dir
                

            if self.is_near_wall == 1:       #近くに壁がある場合
                twist.linear.x = -self.speed #/ 2
            elif self.is_near_wall == -1:
                twist.linear.x = 0
                if self.cruise_dir == 1:
                    twist.angular.z = -0.3
                elif self.cruise_dir == -1:
                    twist.angular.z = 0.3
            self.vel_pub.publish(twist)
            # for debug
            print("POSE: {}, {}".format(self.pose_x, self.pose_y))
            print("POSE TWIST: {}, {}".format(self.pose_twist.linear.x, self.pose_twist.angular.z))
            print("ENEMY TWIST: {}, {}".format(self.near_enemy_twist.linear.x, self.near_enemy_twist.angular.z))
            print("wall: {}, Enemy: {}".format(self.is_near_wall, self.is_near_enemy))     

            r.sleep()
            



if __name__ == '__main__':
    rospy.init_node('Navirun')
    bot = ChiliCheeseBurger()
    bot.strategy()