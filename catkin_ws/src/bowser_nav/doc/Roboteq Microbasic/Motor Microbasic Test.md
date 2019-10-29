# Motor Speed Test
This is a test to see what the parameters in MicroBasic on the Roboteq Utility program, on Windows, means.

| Motor control | Motor Power | Speed 2 RPM | Speed 2 % | Large | Small
|---|---|---|---|---|---
| 118 | 118 | 495-510 | 49.5-51 | 9 RPM | N/A
| 196 | 196 | 1005-10020 | 100.5-102 | 18 RPM | N/A
| N/A | N/A | N/A | 150 | 26.5 RPM | N/A

|Input | Output | Formula*
|---|---|---
|speed %| Large RPM|Large RPM = 0.18*(speed %)

*This test is done without load (weight). We will want to repeat this test to verify the validity of this formula using a range of weights for the load.
