function [curvature] = get_curvature(traj, theta)
    
    dphi = ppval(traj.dppx,theta);
    ddphi = ppval(traj.ddppx,theta);
    domega = ppval(traj.dppy,theta);
    ddomega = ppval(traj.ddppy,theta);
    numerator = abs(dphi * ddomega - domega * ddphi);
    denominator = sqrt((dphi ^ 2 + domega ^2) ^ 3);
    curvature = numerator / denominator;