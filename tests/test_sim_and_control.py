import numpy as np
from src.robot import RRRRobot
from src.sim   import Target

def test_target_period():
    targ = Target(L=1.0, f=0.5)
    pos1, _ = targ.step(2.0)  # two seconds → one full cycle
    assert np.allclose(pos1[1], targ.L * np.sin(2*np.pi*targ.f*2.0))

def test_jacobian_shape():
    robot = RRRRobot()
    J = robot.jacobian([0.1, 0.2, 0.3])
    assert J.shape == (2,3)

def test_run_smoke():
    # import your run function via command-line mode
    import subprocess
    for mode in ("fast","slow","obstacle"):
        ret = subprocess.run(
            ["python3", "src/main.py", "--mode", mode, "--duration", "0.1"],
            stdout=subprocess.PIPE
        )
        assert ret.returncode == 0
