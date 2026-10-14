from pathlib import Path
from pxr import Usd
P=Path(__file__).resolve().parents[1]
if __name__=='__main__':
    s=Usd.Stage.Open(str(P/'assets/scene3_g1.usda'))
    s.Flatten().Export(str(P/'assets/scene3_g1_standalone.usdc'))
    print('Exported standalone scene with G1. Cups are empty; run_pour.py creates water.')
