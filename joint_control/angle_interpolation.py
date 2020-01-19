'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
import numpy as np



class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.keyframeStartTime = 0
        self.keyframeDone = 1
        self.keyframeStartPosition = []

   def set_keyframes(self, keyframes, interrupt = 0):
        if self.keyframeDone or interrupt:
            self.keyframeStartTime = self.perception.time
            self.keyframes = kf

            names, times, keys = kf
            self.keyframeStartPosition = []

            for i,name in enumerate(names):
                firstKeyframeTime = times[i][0]
                keyframeTimeDiff = keys[i][0][1][1]

                if(name in self.perception.joint):
                    self.keyframeStartPosition.append([self.perception.joint[name], [3,0,0], [3,-keyframeTimeDiff,0]])

                else:
                    self.keyframeStartPosition.append([0, [3,0,0], [3,-keyframeTimeDiff,0]])

            self.keyframeDone = 0

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        (names, times, keys) = keyframes
        time = self.perception.time - self.keyframeStartTime
        finished = 1
        for i,name in enumerate(names):
            curTime = times[i]
             #skip if outside of timeframe
            if curTime[-1] < time:
                continue

            finished = 0
            ###Determine point 0(x,y)
            endIdx = len([i for i in curTime if i<time])

            if endIdx != 0:
                startIndex = endIdx - 1
                startKey = keys[i][startIndex]
                p0X = currTimes[startIndex]
            elif endIdx == 0:
                startKey = self.keyframeStartPosition[i]
                p0X = 0

            point0Y = startKey[0]


            ###Use point 0(x,y) to determine point 1(x,y)
            (p1X, p1Y) = (p0X + startKey[2][1], p0Y + startKey[2][2])

            ###Determine point 3(x,y)
            endK = keys[i][endIdx]
            (p3X, p3Y) = (currTimes[endIdx], endK[0])

            ###Use point 3 to determine point 2(x,y)
            (point2X, point2Y) = (point3X + endK[1][1], point3Y + endKey[1][2])

            ##Bezier polynomial
            bezierMatrix = np.array([[1,0,0,0] , [-3,3,0,0] , [3, -6,3,0] , [-1,3,-3,1]])


            pX = np.array([p0X, p1X, p2X, p3X])
            pY = np.array([p0Y, p1Y, p2Y, p3Y])

            ##Finding solutions for the polynomial using Time
            coefX = np.dot(bezierMatrix, pX)
            coefX[0] -= time
            kin_sol = np.polynomial.polynomial.polyroots(coefX)

            ##Solutions in [0,1]
            kin_sol = [x.real for x in kin_sol if -(epsilon) <= x.real <= 1+(epsilon) and x.imag == 0]

            ##we wanna find the right kinematic solution here
            ###if we have more than 1 then lets choose the one closest to 1/2
            if len(kin_sol) > 1:
                kin_sol = np.asarray([(x,np.abs(x-0.5)) for x in kin_sol], dtype = [("value", float),("distance", float)])
                kin_sol = np.sort(solutions, order = "distance")
                realSol = kin_sol[0][0]

            ###If only one solution
            else:
                realSol = kin_sol[0]

            ###Correct for marginal error
            if realSol < 0.:
                realSol = 0.

            if realSol > 1.:
                realSol = 1.

            ##Y values
            coefY = np.dot(bezierMatrix, pY)

            #Final results
            result = np.dot(np.array([1, realSol, realSol**2, realSol**3]), coefY)
            target_joints[name] = result

        self.keyframeDone = done
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
