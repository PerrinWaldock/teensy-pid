from skopt import gp_minimize
#TODO try minimizing stability score using gp_minimize

if __name__ == "__main__":    
    from pidTester import PidTester, PidController, calcStabilityScore

    # parser = argparse.ArgumentParser()
    # parser.add_argument("-s")
    
    pc = PidController()
    pt = PidTester(pc)
    
    def getScore(p):
        print(p)
        kp, ki = p
        pc.kp = kp
        pc.ki = ki
        return calcStabilityScore(pt, num = 100)
    
    kirange = [30,50000.0]
    kprange = [0.0003,.5]
    
    res = gp_minimize(getScore,
                      (kprange, kirange),
                      n_calls=200,
                      verbose=True)
    print(res.x)