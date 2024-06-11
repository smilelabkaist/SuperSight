# SuperSight Localization Tool
Welcome to SuperSight localization tool.
This is the source code for MobiSys '24, SuperSight: Sub-cm NLOS Localization for mmWave Backscatter [[1]](#1)
<details><summary> <strong>Cite this tool</strong>
</summary>

  ### Plain Text
  ```
Kang Min Bae, Hankyeol Moon, and Song Min Kim. 2024. SuperSight: Sub-cm NLOS Localization for mmWave Backscatter. In Proceedings of the 22nd Annual International Conference on Mobile Systems, Applications and Services (MobiSys '24). Association for Computing Machinery, New York, NY, USA, 278–291. https://doi.org/10.1145/3643832.3661857
```
 ### BibTex
 
 ```
@inproceedings{10.1145/3643832.3661857,
  title = {SuperSight: Sub-cm NLOS Localization for mmWave Backscatter},
  author = {Bae, Kang Min and Moon, Hankyeol and Kim, Song Min},
  booktitle = {Proceedings of the 22nd Annual International Conference on Mobile Systems, Applications and Services},
  pages = {278–291},
  year = {2024}
}  
  ```
</details>

## SuperSight NLOS Algorithm
The SuperSight localization tool consists of a single code: SuperSightNLOSAlgorithm.m, which takes the *time-of-flight* and the *angle-of-arrival* of the three paths as input, and ouputs the SuperSight localization result.

The setup details including the *inter-tag spacing* of the triangular tag array, as well as the *position of each radars* (for multi-radar setup) must be defined accordingly in the code.

From given inputs, the code solves the system of equations to deterministically solve for the tag 6DoF.

If you apply our tool in your research, please cite our paper [[1]](#1). For troubleshooting, please contact the SMILE LAB (smilelabkaist@gmail.com).

## License
This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.

## Reference
<a id="1">[1]</a> 
Kang Min Bae, Hankyeol Moon, and Song Min Kim. "SuperSight: Sub-cm NLOS Localization for mmWave Backscatter", MobiSys '24.
