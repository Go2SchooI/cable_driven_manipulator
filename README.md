## **Joint space to Cartesian space**

Common manipulators kinematics demo based on matlab, including forward kinematics, inverse kinematics, Jacobian matrices, etc. The code is abbreviated as it is only used for feasibility checking and will be optimised at a later stage.

------------------------------------------------------

### Version 3.0

gpm_k = 0.015;  	learning_ratio = 0.2; 收敛效果较差，learning_ratio = 0.3时收敛效果极差

gpm_k = 0.015;  	learning_ratio = 0.15; 收敛效果较好

gpm_k = 0.015;  	learning_ratio = 0.08; 收敛速度较慢，增加最大迭代次数后，收敛精度与0.15类似

以上gpm_k对角度限制效果不够

gpm_k = 0.15，能起到较明显作用，但是对其余关节角影响大

------------------

### Version 2.1

Supplement KUKA LBR iiwa manipulator demo, completing inverse kinematics based on gradient projection method (GPM).

Assume $$\theta5$$ has max as pi/2 and min as pi/3.

<img src="https://cdn.jsdelivr.net/gh/Go2SchooI/blogImg@main/img/image-20240130201539629.png" alt="image-20240130201539629" style="zoom:50%;" />

<table style="border:none;text-align:center;width:auto;margin: 0 auto;">
        <tr>
            <td style="border: none;"><img src = "https://cdn.jsdelivr.net/gh/Go2SchooI/blogImg@main/img/image-20240130211559690.png" alt="target" style="zoom:100%;"></td>
            <td style="border: none;"><img src = "https://cdn.jsdelivr.net/gh/Go2SchooI/blogImg@main/img/image-20240130201618325.png" alt="dls_result" style="zoom:100%;"></td>
    	</tr>
        <tr>
            <td><div style="font-family:黑体;font-size:8pt;">Figure 1 Target Angle</div></td><td><div style="font-family:黑体;font-size:8pt;">Figure 2 Result Angle</div></td>
    </tr>
</table>


We can see that $$\theta5$$ is limited.

-------------------------------------------

### Version 2.0

Based on the purpose of effect visualisation, function encapsulation other than inverse kinematics is completed.

Add KUKA LBR iiwa manipulator demo, completing inverse kinematics based on Damped Least Square (DLS) method.

<table style="border:none;text-align:center;width:auto;margin: 0 auto;">
        <tr>
            <td style="border: none;"><img src = "https://cdn.jsdelivr.net/gh/Go2SchooI/blogImg@main/img/target.png" alt="target" style="zoom:33%;"></td>
            <td style="border: none;"><img src = "https://cdn.jsdelivr.net/gh/Go2SchooI/blogImg@main/img/dls_result.png" alt="dls_result" style="zoom:33%;"></td>
    	</tr>
        <tr>
            <td><div style="font-family:黑体;font-size:8pt;">Figure 1 Target 6D Pose</div></td><td><div style="font-family:黑体;font-size:8pt;">Figure 2 Result 6D Pose</div></td>
    </tr>
</table>


----------------------------------------------------------------------------------------------------------------------------------------

### Version 1.0

Algorithm validation using PUMA560 as an example, including forward kinematics, inverse kinematics, Jacobi matrices, etc. The code is abbreviated as it is only used for feasibility checking and will be optimised at a later stage.



Error design:

e(1:3) is the translation error, which can be calculated from $x_{target} - x$.

e(4:6) is the rotation error, described by the rotation matrix. 

Suppose there is a vector called $A_1$ in coordinate system 1, $A_2$ in coordinate system 2, then:

$$
A_2=C_1^2A_1
$$

For the same reason:

$$
A_3=C_2^3 A_2
$$

Then

$$
A_3=C_2^3 C_1^2 A_1
$$

So

$$
C_1^3=C_2^3 C_1^2
$$

This formula describes the conversion of two rotations added together into one larger rotation. That is, the addition of rotations.

So

$$
C_2^{3^{-1}} C_1^3=C_1^2
$$

This would indicate that a large rotation minus a small rotation equals another rotation. That is, subtraction between rotations.

In matrix operation, "inverse" is not easy to find, the best matrix to find is "transpose". But since the rotated matrix is "orthogonal matrix", its "transpose" = "inverse", so not only the error of the rotated matrix must be a rotated matrix, but also the calculation process becomes very simple.

$$
C_2^{3^{T}} C_1^3=C_1^2
$$




## **References**

[1] Peter Corke, MATLAB Robotics Toolbox [http://petercorke.com](http://petercorke.com/).

[2] [SJTU-RoboMaster-Team](https://github.com/SJTU-RoboMaster-Team), [Matrix_and_Robotics_on_STM32](https://github.com/SJTU-RoboMaster-Team/Matrix_and_Robotics_on_STM32)

[3] J.J.Craig, Introduction to robotics mechanics and control

[4] Buss, S. R. (2004). Introduction to inverse kinematics with Jacobian transpose, pseudoinverse and damped least squares methods. *IEEE Journal of Robotics & Automation, 17*(1).

[5] Chiaverini, S., Oriolo, G., & Maciejewski, A. A. (2016). *Redundant Robots*. Cham, Switzerland: Springer.

[6] Woliński, Ł., & Wojtyra, M. (2022). A Novel QP-Based Kinematic Redundancy Resolution Method With Joint Constraints Satisfaction. *IEEE Access, 10*, 41023-41037. doi: 10.1109/ACCESS.2022.3167403.