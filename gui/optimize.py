from skopt import gp_minimize
from pidTester import PidTester, PidController, calcStabilityScore

#TODO try minimizing stability score using gp_minimize

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    def getScore(p):
        print(p)
        kp, ki = p
        pc.kp = kp
        pc.ki = ki
        return calcStabilityScore(pt)
    
    kirange = [30,50000.0]
    kprange = [0.0003,.5]
    
    res = gp_minimize(getScore,
                      (kprange, kirange),
                      n_calls=100,
                      verbose=True)
    print(res.x)