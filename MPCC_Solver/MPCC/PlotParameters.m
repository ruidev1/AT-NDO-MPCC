function [ ] = PlotParameters(MPC_vars, simN, n, i)
    
%     N = MPC_vars.N;
    i = i - 1;
    Ts = MPC_vars.Ts;
    figure(4)
    params = [MPC_vars.qC; MPC_vars.qVtheta; MPC_vars.qL];
    param_remap = repmat(params,1,simN+1);
    subplot(n,1,1)
    plot([simN * i: simN * (i+1)]*Ts,param_remap(1,:))
    hold on
    xlabel('time [s]')
    ylabel('qC [-]')
    ylim([-0.1 10.1])
%     yticks(-0.1: 0.1: 2.5)
    subplot(n,1,2)
    plot([simN * i: simN * (i+1)]*Ts,param_remap(2,:))
    hold on
    xlabel('time [s]')
    ylabel('qVtheta [-]')
    ylim([-0.1 1.1])
%     yticks(-0.1: 0.1: 1.5)
    subplot(n,1,3)
    plot([simN * i: simN * (i+1)]*Ts,param_remap(3,:))
    hold on
    xlabel('time [s]')
    ylabel('qL [-]')
    ylim([490 1510])
%     subplot(n,1,3)
%     plot([0:simN]*Ts,Action_remap(3,:))
%     xlabel('time [s]')
%     ylabel('qOmega [-]')
%     subplot(n,1,4)
%     plot([0:simN]*Ts,Action_remap(4,:))
%     xlabel('time [s]')
%     ylabel('rVtheta [-]')

end