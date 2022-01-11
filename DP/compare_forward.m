classdef Compare_DPandNMPC < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure              matlab.ui.Figure
        GridLayout            matlab.ui.container.GridLayout
        LeftPanel             matlab.ui.container.Panel
        UIAxes                matlab.ui.control.UIAxes
        UIAxes_2              matlab.ui.control.UIAxes
        UIAxes_3              matlab.ui.control.UIAxes
        CenterPanel           matlab.ui.container.Panel
        Label                 matlab.ui.control.Label
        UIAxes3               matlab.ui.control.UIAxes
        UIAxes2               matlab.ui.control.UIAxes
        RightPanel            matlab.ui.container.Panel
        Pm2TextAreaLabel      matlab.ui.control.Label
        Pm2TextArea           matlab.ui.control.TextArea
        Pe2TextAreaLabel      matlab.ui.control.Label
        Pe2TextArea           matlab.ui.control.TextArea
        PmTextArea_2Label     matlab.ui.control.Label
        PmTextArea_2          matlab.ui.control.TextArea
        PeTextAreaLabel       matlab.ui.control.Label
        PeTextArea            matlab.ui.control.TextArea
        kWLabel_2             matlab.ui.control.Label
        kWLabel               matlab.ui.control.Label
        kWsLabel_2            matlab.ui.control.Label
        kWsLabel              matlab.ui.control.Label
        sLabel_2              matlab.ui.control.Label
        sLabel                matlab.ui.control.Label
        kmhLabel_2            matlab.ui.control.Label
        kmhLabel              matlab.ui.control.Label
        Speed2TextArea        matlab.ui.control.TextArea
        Speed2TextAreaLabel   matlab.ui.control.Label
        Time2TextArea         matlab.ui.control.TextArea
        Time2TextAreaLabel    matlab.ui.control.Label
        Energy2TextAreaLabel  matlab.ui.control.Label
        Energy2TextArea       matlab.ui.control.TextArea
        Eff2TextAreaLabel     matlab.ui.control.Label
        Eff2TextArea          matlab.ui.control.TextArea
        Position2Label        matlab.ui.control.Label
        Spinner_2             matlab.ui.control.Spinner
        SpeedTextArea         matlab.ui.control.TextArea
        SpeedTextAreaLabel    matlab.ui.control.Label
        Position1Label        matlab.ui.control.Label
        Spinner               matlab.ui.control.Spinner
        OptionButton          matlab.ui.control.Button
        DrivingButton         matlab.ui.control.Button
        T0EditField           matlab.ui.control.NumericEditField
        T0EditFieldLabel      matlab.ui.control.Label
        V0EditField           matlab.ui.control.NumericEditField
        V0EditFieldLabel      matlab.ui.control.Label
        FbEditField           matlab.ui.control.NumericEditField
        FbEditFieldLabel      matlab.ui.control.Label
        FmEditField           matlab.ui.control.NumericEditField
        FmEditFieldLabel      matlab.ui.control.Label
        EnergyTextAreaLabel   matlab.ui.control.Label
        EnergyTextArea        matlab.ui.control.TextArea
        EffTextAreaLabel      matlab.ui.control.Label
        EffTextArea           matlab.ui.control.TextArea
        TimeTextArea          matlab.ui.control.TextArea
        TimeTextAreaLabel     matlab.ui.control.Label
    end

    % Properties that correspond to apps with auto-reflow
    properties (Access = private)
        onePanelWidth = 576;
        twoPanelWidth = 768;
    end

    properties (Access = private)
        % 외부앱 정의
        Data_selectApp         % Dialog box app
        % Vehicle Parameter
        coef = load('Estimation/Data_mat/coefficient.mat');
        MC = load('Estimation/Data_mat/motor_spec.mat');
        VP = load('Estimation/Data_mat/Parameter_init.mat');
        % 설정변수
        DP1; NMPC2; scen=1;
        lim_force
        P_elec;
        ubr;
        flag;
        
        
        %Forward 계산 변수
        sel_x; sel_x2;
        sel_t; sel_t2;
        Pe; Pe2;
        Pe_v; Pe_v2;
        Pm; Pm2;
        Ee; Ee2;
        Em; Em2;
        eff; eff2;
        opt_total_input; opt_total_input2;
        opt_Fm_input; opt_Fm_input2;
        opt_Fb_input; opt_Fb_input2;
        pos;pos2;
        lim_trqt;lim_trqt2;
        
        
    end
    
    methods (Access = public)
        function updatedata(app, DP_1,NMPC_2 ,flag)
            app.DP1 = load(['Nonlinear/DP_RES/' DP_1]);
            app.Spinner.Step = app.DP1.Ss;
            app.ubr = (-6000:app.FbEditField.Value:0)';
            app.P_elec = @(v,f) (1/v).*(v*app.coef.Pc(1)+v.*f*app.coef.Pc(2)+v.*f.^2*app.coef.Pc(3)+v.^3*app.coef.Pc(4)...
                +v.^3.*f*app.coef.Pc(5)+v.^5*app.coef.Pc(6));
            app.lim_force = @(v) [1 1./v]*app.coef.c1;
            app.flag = flag;
            app.Label.Text = string(['S_{1,f} = ',num2str(app.DP1.Ss*(app.DP1.N-1)), ',  lam_1 = ',num2str(app.DP1.lam)]);
            if flag==1
                app.NMPC2 = load(['MPC_RES/' NMPC_2]);
                app.Spinner_2.Step = app.NMPC2.Ss;
                app.Label.Text = [string(['S_{1,f} = ',num2str(app.DP1.Ss*(app.DP1.N-1)), ',  lam_1 = ',num2str(app.DP1.lam)]);...
                    string(['S_{2,f} = ',num2str(app.NMPC2.Ss*(app.NMPC2.N-1)), ',  lam_2 = ',num2str(app.NMPC2.lam)])];
            end
            fprintf('데이터 로드 완료\n')
        end
        
        function forwardDP(app)
            % forward DP Compute - 만약 flag = 1이라면 NMPC랑 같이 비교함. 일반적으로는 아님.
            app.sel_x = zeros(app.DP1.N,1);
            app.sel_t = zeros(app.DP1.N,1);
            app.Pe = zeros(app.DP1.N-1,1);
            app.Pe_v = zeros(app.DP1.N-1,1);
            app.Pm = zeros(app.DP1.N-1,1);
            app.eff = zeros(app.DP1.N-1,1);
            app.opt_total_input = zeros(app.DP1.N-1,1);
            app.opt_Fm_input = zeros(app.DP1.N-1,1);
            app.opt_Fb_input = zeros(app.DP1.N-1,1);
            app.sel_x(1,1) = app.V0EditField.Value;
            app.sel_t(1,1) = app.T0EditField.Value;
            if app.flag ==1 % NMPC랑 비교할때.
                app.sel_x2 = app.NMPC2.x_opt(:,1);
                app.sel_t2 = cumtrapz(1./app.sel_x2);
                app.Pe2 = zeros(app.NMPC2.N-1,1);
                app.Pe_v2 = zeros(app.NMPC2.N-1,1);
                app.Pm2 = zeros(app.NMPC2.N-1,1);
                app.eff2 = zeros(app.NMPC2.N-1,1);
                app.opt_total_input2 = (app.NMPC2.u_opt(:,1)+app.NMPC2.u_opt(:,2))';
                app.opt_Fm_input2 = app.NMPC2.u_opt(:,1)';
                app.opt_Fb_input2 = app.NMPC2.u_opt(:,2)';
            end
            for i=1:app.DP1.N-1
                % 변수정의
                app.lim_trqt= min(app.coef.const_max_force,app.lim_force(app.sel_x(i)));
                ut = (-app.lim_trqt:app.FmEditField.Value:app.lim_trqt)'; % torque without brake
                ubt = [app.ubr-app.lim_trqt;ut]; % torque with brake (added above)
                uct = [app.ubr;zeros(size(ut,1),1)]; % brake torque only and zeros below
                ut = [-app.lim_trqt*ones(size(app.ubr,1),1);ut];
                
                f = @(t,x) 1./abs(x).*(ubt/(app.VP.m) - app.VP.Ca*x.^2  - app.VP.g*(app.VP.Cr*cos(app.DP1.slop(i))+sin(app.DP1.slop(i))));
                [aa,x_next_ode45_t] = ode45(f,[0 app.DP1.Ss/2 app.DP1.Ss],app.sel_x(i,1)*ones(size(ut)));
                x_next = x_next_ode45_t(end,:)';
                
                delta_t = 2*app.DP1.Ss./(app.sel_x(i,1)+x_next);
                t_next = app.sel_t(i,1) + delta_t';
                
                Jt = interp2(app.DP1.t,app.DP1.x,app.DP1.J{i+1,1},t_next',x_next);
                [JJ,ind_input] = min(app.DP1.lam*app.DP1.Ss*(app.P_elec(app.sel_x(i,1),ut)-uct*app.sel_x(i,1))+Jt);
                
                app.opt_total_input(i,1) = ubt(ind_input);
                app.opt_Fm_input(i,1) = ut(ind_input); % 모터파워?
                app.opt_Fb_input(i,1) = uct(ind_input); % 브레이크힘
                
                app.Pe_v(i) = app.P_elec(app.sel_x(i),app.opt_Fm_input(i));
                app.Pe(i)  = app.Pe_v(i)*app.sel_x(i); % 모터에서 소모하는 전기적 에너지
                app.Pm(i) = app.sel_x(i)*app.opt_Fm_input(i); % % 모터 기계적 에너지
                
                % 에너지 효율 계산
                if abs(app.Pm(i))<abs(app.Pe(i)) % 방전할때 (모터로써 쓸때)
                    app.eff(i) = app.Pm(i)./app.Pe(i); % 기계적에너지/전기적에너지 : 전기적에너지 -> 기계적에너지 변환효율
                else % 충전할때 (모터가 아닌 발전기로서 회생제동)
                    app.eff(i) = app.Pe(i)./app.Pm(i); % 전기적에너지/기계적에너지 : 기계적에너지 -> 전기적에너지 변환효율
                end
                
                
                f = @(t,x) 1./abs(x).*(app.opt_total_input(i,1)/(app.VP.m) - app.VP.Ca*x.^2  - app.VP.g*(app.VP.Cr*cos(app.DP1.slop(i))+sin(app.DP1.slop(i))));
                [aa,sel_xt] = ode45(f,[0 app.DP1.Ss/2 app.DP1.Ss],app.sel_x(i,1));
                app.sel_x(i+1,1) = sel_xt(end); % 다음 step에서의 속도
                app.sel_t(i+1,1) = app.sel_t(i,1)+2*app.DP1.Ss/(app.sel_x(i+1,1)+app.sel_x(i,1)); % 다음 step에서의 시간
                %{
                                 if app.sel_x(i+1,1)<app.v1(1) || app.sel_x(i+1,1)>app.v1(end)
                                     error('속도가 Quantization내에 존재하지 않습니다.')
                                 elseif app.sel_t(i+1,1)<app.t1(1) || app.sel_t(i+1,1)>app.t1(end)
                                     error('시간이 Quantization내에 존재하지 않습니다.')
                                 end
                %}
                
            end          
            
            app.Ee = cumtrapz(app.DP1.dist(1:end-1),app.Pe_v);
            app.Em = cumtrapz(app.DP1.dist(1:end-1),app.opt_Fm_input);
            app.Spinner.Value = 0;
            app.pos = 0;
            app.Spinner.Limits = [0,app.DP1.N*app.DP1.Ss-1];
            if app.flag == 1
                for i=1:app.NMPC2.N-1
                    app.Pe_v2(i) = app.P_elec(app.sel_x2(i),app.opt_Fm_input2(i));
                    app.Pe2(i)  = app.Pe_v2(i)*app.sel_x2(i);
                    app.Pm2(i) = app.sel_x2(i)*app.opt_Fm_input2(i);
                    if abs(app.Pm2(i))<abs(app.Pe2(i))
                        app.eff2(i) = app.Pm2(i)./app.Pe2(i);
                    else
                        app.eff2(i) = app.Pe2(i)./app.Pm2(i);
                    end
                end
                app.Ee2 = cumtrapz(app.NMPC2.dist(1:end-1),app.Pe_v2);
                app.Em2 = cumtrapz(app.NMPC2.dist(1:end-1),app.opt_Fm_input2);
                app.Spinner_2.Value = 0;
                app.pos2 = 0;
                app.Spinner_2.Limits = [0,app.NMPC2.N*app.NMPC2.Ss-1];
            end
            
            fprintf('완료\n');
            stateplot(app);
            Inputplot(app);
        end
        
        function Inputplot(app)
            if app.pos == (app.DP1.N-1)
                hold(app.UIAxes2,'off');
                stairs(app.UIAxes2,app.DP1.dist(1:app.pos),app.opt_total_input(1:app.pos),'g','LineWidth',2)
                hold(app.UIAxes2,'on');grid(app.UIAxes2,'on')
                stairs(app.UIAxes2,app.DP1.dist(1:app.pos),app.opt_Fm_input(1:app.pos),'r--','LineWidth',2)
                stairs(app.UIAxes2,app.DP1.dist(1:app.pos),app.opt_Fb_input(1:app.pos),'b--','LineWidth',2)
                xlabel(app.UIAxes2,'Distance [m]')
                ylabel(app.UIAxes2,'Motor driven Wheel force^* [N]')
                legend(app.UIAxes2,'F_{total}','F_m','F_b')
                
                hold(app.UIAxes3,'off');
                contour(app.UIAxes3,app.VP.mc_map_vel,app.VP.mc_map_force,app.VP.mc_eff_map','ShowText','on','LabelSpacing',600);
                hold(app.UIAxes3,'on');
                p1=plot(app.UIAxes3,app.MC.vt,app.MC.lim_fd,app.MC.vt,-app.MC.lim_fd);          
                p1(1).LineWidth = 2; p1(2).LineWidth = 2;
                p1(1).Color = 'k'; p1(2).Color = 'k';
                plot(app.UIAxes3,app.sel_x(1:app.pos),app.opt_Fm_input(1:app.pos),'ro','MarkerFaceColor','r','LineWidth',2)
                %                 xlim(app.UIAxes3,[0,app.DP1.x(end)]);
                xlabel(app.UIAxes3,'Speed [m/s]')
                ylabel(app.UIAxes3,'Motor driven Wheel force^* [N]')
            else
                hold(app.UIAxes2,'off');
                stairs(app.UIAxes2,app.DP1.dist(1:app.pos+1),app.opt_total_input(1:app.pos+1),'g','LineWidth',2)
                hold(app.UIAxes2,'on');grid(app.UIAxes2,'on')
                stairs(app.UIAxes2,app.DP1.dist(1:app.pos+1),app.opt_Fm_input(1:app.pos+1),'r--','LineWidth',2)
                stairs(app.UIAxes2,app.DP1.dist(1:app.pos+1),app.opt_Fb_input(1:app.pos+1),'b--','LineWidth',2)
                xlabel(app.UIAxes2,'Distance [m]')
                ylabel(app.UIAxes2,'Motor driven Wheel force^* [N]')
                legend(app.UIAxes2,'F_{total}','F_m','F_b')
                
                hold(app.UIAxes3,'off');
                contour(app.UIAxes3,app.VP.mc_map_vel,app.VP.mc_map_force,app.VP.mc_eff_map','ShowText','on','LabelSpacing',600);
                hold(app.UIAxes3,'on');grid(app.UIAxes3,'on')
                p1=plot(app.UIAxes3,app.MC.vt,app.MC.lim_fd,app.MC.vt,-app.MC.lim_fd);
                p1(1).LineWidth = 2; p1(2).LineWidth = 2;
                p1(1).Color = 'k'; p1(2).Color = 'k';
                plot(app.UIAxes3,app.sel_x(1:app.pos+1),app.opt_Fm_input(1:app.pos+1),'ro','MarkerFaceColor','r','LineWidth',2)
                xlabel(app.UIAxes3,'Speed [m/s]')
                ylabel(app.UIAxes3,'Motor driven Wheel force^* [N]')
%                 xlim(app.UIAxes3,[0,app.DP1.x(end)]);
            end
            if app.flag == 1
                if app.pos2 == (app.NMPC2.N-1)
                    stairs(app.UIAxes2,app.NMPC2.dist(1:app.pos2),app.opt_total_input2(1:app.pos2),'k-','LineWidth',2)
                    stairs(app.UIAxes2,app.NMPC2.dist(1:app.pos2),app.opt_Fm_input2(1:app.pos2),'r-.','LineWidth',2)
                    stairs(app.UIAxes2,app.NMPC2.dist(1:app.pos2),app.opt_Fb_input2(1:app.pos2),'b-.','LineWidth',2)
                    legend(app.UIAxes2,'F_{total}','F_m','F_b','F_{total2}','F_{m2}','F_{b2}')
                    plot(app.UIAxes3,app.sel_x2(1:app.pos2),app.opt_Fm_input2(1:app.pos2),'bo','MarkerFaceColor','b','LineWidth',2)
                else
%                     fprintf('Pe/v : %d, Pe : %d, v : %d Em : %d \n',app.Pe_v2(app.pos2+1),app.Pe2(app.pos2+1),app.sel_x2((app.pos2+1)),app.Ee2(app.pos2+1));
                    stairs(app.UIAxes2,app.NMPC2.dist(1:app.pos2+1),app.opt_total_input2(1:app.pos2+1),'k-','LineWidth',2)
                    stairs(app.UIAxes2,app.NMPC2.dist(1:app.pos2+1),app.opt_Fm_input2(1:app.pos2+1),'r-.','LineWidth',2)
                    stairs(app.UIAxes2,app.NMPC2.dist(1:app.pos2+1),app.opt_Fb_input2(1:app.pos2+1),'b-.','LineWidth',2)
                    legend(app.UIAxes2,'F_{total}','F_m','F_b','F_{total2}','F_{m2}','F_{b2}')
                    plot(app.UIAxes3,app.sel_x2(1:app.pos2+1),app.opt_Fm_input2(1:app.pos2+1)','bo','MarkerFaceColor','b','LineWidth',2)
                end
                
            end
        end
        
        function stateplot(app)
            switch app.scen
                case 1
                    plot(app.UIAxes,app.DP1.dist(1:app.pos+1),app.sel_x(1:app.pos+1),'r','LineWidth',2)
                    app.UIAxes.XGrid = 'on';
                    app.UIAxes.YGrid = 'on';
                    % xlabel('Time index')
                    ylabel(app.UIAxes,'velocity^* [m/s]')
                    
                    plot(app.UIAxes_2,app.DP1.dist(1:app.pos+1),app.sel_t(1:app.pos+1),'r','LineWidth',2)
                    grid(app.UIAxes_2,'on')
                    xlabel(app.UIAxes2,'Distance [m]')
                    ylabel(app.UIAxes2,'Time^* [m/s]')
                    % xlabel('Time index')
                    
                    if app.pos == (app.DP1.N-1)
                        plot(app.UIAxes_3,app.DP1.dist(1:app.pos),(app.DP1.hight(1:app.pos)),'r','LineWidth',2)
                        grid(app.UIAxes_3,'on')
                        xlabel(app.UIAxes_3,'Distance [m]')
                        ylabel(app.UIAxes_3,'Road slope [m]')
                    else
                        plot(app.UIAxes_3,app.DP1.dist(1:app.pos+1),(app.DP1.hight(1:app.pos+1)),'r','LineWidth',2)
                        grid(app.UIAxes_3,'on')
                        xlabel(app.UIAxes_3,'Distance [m]')
                        ylabel(app.UIAxes_3,'Road slope [m]')
                    end
                    %plot(app.sel_t(1:app.pos/app.Ss+1),app.sel_x(1:app.pos/app.Ss+1),'LineWidth',2)
                    if app.flag == 1
                        hold(app.UIAxes,'on')
                        plot(app.UIAxes,app.NMPC2.dist(1:app.pos2+1),app.sel_x2(1:app.pos2+1),'b-.','LineWidth',2)
                        title(app.UIAxes,[string(['S = ',num2str(app.pos2)]);...
                            string(['S_f = ',num2str(app.NMPC2.Ss*(app.NMPC2.N-1)),',  lam = ',num2str(app.NMPC2.lam)])]);
                        % xlabel('Time index')
                        ylabel(app.UIAxes,'velocity^* [m/s]')
                        hold(app.UIAxes,'off')
                        
                        hold(app.UIAxes_2,'on')
                        plot(app.UIAxes_2,app.NMPC2.dist(1:app.pos2+1),app.sel_t2(1:app.pos2+1),'b-.','LineWidth',2)
                        xlabel(app.UIAxes_2,'Distance [m]')
                        ylabel(app.UIAxes_2,'Time^* [m/s]')
                        % xlabel('Time index')
                        hold(app.UIAxes_2,'off')
                        
                        hold(app.UIAxes_3,'on')
                        if app.pos == (app.NMPC2.N-1)
                            plot(app.UIAxes_3,app.NMPC2.dist(1:app.pos2),(app.NMPC2.hight(1:app.pos2)),'b-.','LineWidth',2)
                            xlabel(app.UIAxes3,'Distance [m]')
                            ylabel(app.UIAxes3,'Road slope [m]')
                        else
                            plot(app.UIAxes_3,app.NMPC2.dist(1:app.pos2+1),(app.NMPC2.hight(1:app.pos2+1)),'b-.','LineWidth',2)
                            xlabel(app.UIAxes3,'Distance [m]')
                            ylabel(app.UIAxes3,'Road slope [m]')
                        end
                        hold(app.UIAxes_3,'off')
                    end
                case 2
                    plot(app.UIAxes,(0:app.Ss:app.pos),app.sel_x(1:app.pos+1),'LineWidth',2)
                    title(app.UIAxes,[string(['S = ',num2str(app.pos*app.DP1.Ss)]);...
                        string(app.UIAxes,['S_f = ',num2str(app.DP1.Ss*(app.N-1)),',  V_f = ',num2str(app.vf),...
                        ',  T_f = ',num2str(app.tf)])]);
                    grid on
                    % xlabel('Time index')
                    ylabel(app.UIAxes,'velocity^* [m/s]')
                    plot(app.UIAxes_2,(0:app.Ss:app.pos),app.sel_t(1:app.pos+1),'LineWidth',2)
                    grid on
                    xlabel(app.UIAxes2,'Distance [m]')
                    ylabel(app.UIAxes2,'Time^* [m/s]')
                    % xlabel('Time index')
                    %plot(app.sel_t(1:app.pos+1),app.sel_x(1:app.pos/app.Ss+1),'LineWidth',2)
                    plot(app.UIAxes_3,(0:app.Ss:app.pos),rad2deg(app.slop(1:app.pos+1)),'LineWidth',2)
                    grid on
                    xlabel(app.UIAxes3,'Distance [m]')
                    ylabel(app.UIAxes3,'Road slope [degree]')
            end
        end
    end

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            %             app.phi = 0.5*app.CdAf*app.rhoa/app.m;
            %             app.mc_eff_map_c = app.Motor.mc_eff_map;
        end

        % Button pushed function: OptionButton
        function OptionButtonPushed(app, event)
            %app.OptionButton.Enable = 'off';
            app.Data_selectApp = Data_select(app);
        end

        % Value changed function: Spinner
        function SpinnerValueChanged(app, event)
            app.pos = app.Spinner.Value/app.DP1.Ss;
            if app.pos == app.DP1.N-1
                app.EffTextArea.Value = num2str(sum(app.eff(1:app.pos))/(app.pos));
                app.EnergyTextArea.Value = num2str(app.Ee(app.pos)/1000);
                app.PmTextArea_2.Value = num2str(app.Pm(app.pos)/1000);
                app.PeTextArea.Value = num2str(app.Pe(app.pos)/1000);
            else
                app.EffTextArea.Value = num2str(sum(app.eff(1:app.pos+1))/(app.pos+1));
                app.EnergyTextArea.Value = num2str(app.Ee(app.pos+1)/1000);
                app.PmTextArea_2.Value = num2str(app.Pm(app.pos+1)/1000);
                app.PeTextArea.Value = num2str(app.Pe(app.pos+1)/1000);
            end
            app.TimeTextArea.Value = num2str(app.sel_t(app.pos+1));
            app.SpeedTextArea.Value = num2str((app.sel_x(app.pos+1)*3.6));
            stateplot(app);
            Inputplot(app);
        end

        % Close request function: UIFigure
        function UIFigureCloseRequest(app, event)
            delete(app.Data_selectApp);
            delete(app);
        end

        % Button pushed function: DrivingButton
        function DrivingButtonPushed(app, event)
            forwardDP(app)
        end

        % Value changed function: SpeedTextArea
        function SpeedTextAreaValueChanged(app, event)
            
            
        end

        % Value changed function: Spinner_2
        function Spinner_2ValueChanged(app, event)
            if app.flag==1
                app.pos2 = app.Spinner_2.Value/app.NMPC2.Ss;
                if app.pos2 == app.NMPC2.N-1
                    app.Eff2TextArea.Value = num2str(sum(app.eff2(1:app.pos2))/(app.pos2));
                    app.Energy2TextArea.Value = num2str(app.Ee2(app.pos2)/1000);
                    app.Pm2TextArea.Value = num2str(app.Pm2(app.pos2)/1000);
                    app.Pe2TextArea.Value = num2str(app.Pe2(app.pos2)/1000);
                else
                    app.Eff2TextArea.Value = num2str(sum(app.eff2(1:app.pos2+1))/(app.pos2+1));
                    app.Energy2TextArea.Value = num2str(app.Ee2(app.pos2+1)/1000);
                    app.Pm2TextArea.Value = num2str(app.Pm2(app.pos2+1)/1000);
                    app.Pe2TextArea.Value = num2str(app.Pe2(app.pos2+1)/1000);
                end
                app.Time2TextArea.Value = num2str(app.sel_t2(app.pos2+1));
                app.Speed2TextArea.Value = num2str((app.sel_x2(app.pos2+1)*3.6));
                stateplot(app);
                Inputplot(app);
            end
        end

        % Changes arrangement of the app based on UIFigure width
        function updateAppLayout(app, event)
            currentFigureWidth = app.UIFigure.Position(3);
            if(currentFigureWidth <= app.onePanelWidth)
                % Change to a 3x1 grid
                app.GridLayout.RowHeight = {545, 545, 545};
                app.GridLayout.ColumnWidth = {'1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 1;
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 3;
                app.RightPanel.Layout.Column = 1;
            elseif (currentFigureWidth > app.onePanelWidth && currentFigureWidth <= app.twoPanelWidth)
                % Change to a 2x2 grid
                app.GridLayout.RowHeight = {545, 545};
                app.GridLayout.ColumnWidth = {'1x', '1x'};
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = [1,2];
                app.LeftPanel.Layout.Row = 2;
                app.LeftPanel.Layout.Column = 1;
                app.RightPanel.Layout.Row = 2;
                app.RightPanel.Layout.Column = 2;
            else
                % Change to a 1x3 grid
                app.GridLayout.RowHeight = {'1x'};
                app.GridLayout.ColumnWidth = {426, '1x', 201};
                app.LeftPanel.Layout.Row = 1;
                app.LeftPanel.Layout.Column = 1;
                app.CenterPanel.Layout.Row = 1;
                app.CenterPanel.Layout.Column = 2;
                app.RightPanel.Layout.Row = 1;
                app.RightPanel.Layout.Column = 3;
            end
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.AutoResizeChildren = 'off';
            app.UIFigure.Position = [100 100 962 545];
            app.UIFigure.Name = 'MATLAB App';
            app.UIFigure.CloseRequestFcn = createCallbackFcn(app, @UIFigureCloseRequest, true);
            app.UIFigure.SizeChangedFcn = createCallbackFcn(app, @updateAppLayout, true);

            % Create GridLayout
            app.GridLayout = uigridlayout(app.UIFigure);
            app.GridLayout.ColumnWidth = {426, '1x', 201};
            app.GridLayout.RowHeight = {'1x'};
            app.GridLayout.ColumnSpacing = 0;
            app.GridLayout.RowSpacing = 0;
            app.GridLayout.Padding = [0 0 0 0];
            app.GridLayout.Scrollable = 'on';

            % Create LeftPanel
            app.LeftPanel = uipanel(app.GridLayout);
            app.LeftPanel.Layout.Row = 1;
            app.LeftPanel.Layout.Column = 1;

            % Create UIAxes_3
            app.UIAxes_3 = uiaxes(app.LeftPanel);
            title(app.UIAxes_3, 'Slop')
            xlabel(app.UIAxes_3, 'X')
            ylabel(app.UIAxes_3, 'Y')
            zlabel(app.UIAxes_3, 'Z')
            app.UIAxes_3.Position = [1 23 414 129];

            % Create UIAxes_2
            app.UIAxes_2 = uiaxes(app.LeftPanel);
            title(app.UIAxes_2, 'Time')
            xlabel(app.UIAxes_2, 'X')
            ylabel(app.UIAxes_2, 'Y')
            zlabel(app.UIAxes_2, 'Z')
            app.UIAxes_2.Position = [1 190 419 145];

            % Create UIAxes
            app.UIAxes = uiaxes(app.LeftPanel);
            title(app.UIAxes, 'Speed')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            zlabel(app.UIAxes, 'Z')
            app.UIAxes.Position = [1 345 412 158];

            % Create CenterPanel
            app.CenterPanel = uipanel(app.GridLayout);
            app.CenterPanel.Layout.Row = 1;
            app.CenterPanel.Layout.Column = 2;

            % Create UIAxes2
            app.UIAxes2 = uiaxes(app.CenterPanel);
            title(app.UIAxes2, 'Input')
            xlabel(app.UIAxes2, 'X')
            ylabel(app.UIAxes2, 'Y')
            zlabel(app.UIAxes2, 'Z')
            app.UIAxes2.Position = [9 289 318 185];

            % Create UIAxes3
            app.UIAxes3 = uiaxes(app.CenterPanel);
            title(app.UIAxes3, 'Input')
            xlabel(app.UIAxes3, 'X')
            ylabel(app.UIAxes3, 'Y')
            zlabel(app.UIAxes3, 'Z')
            app.UIAxes3.Position = [9 39 318 185];

            % Create Label
            app.Label = uilabel(app.CenterPanel);
            app.Label.HorizontalAlignment = 'center';
            app.Label.FontWeight = 'bold';
            app.Label.FontAngle = 'italic';
            app.Label.Position = [18 502 309 34];

            % Create RightPanel
            app.RightPanel = uipanel(app.GridLayout);
            app.RightPanel.Layout.Row = 1;
            app.RightPanel.Layout.Column = 3;

            % Create TimeTextAreaLabel
            app.TimeTextAreaLabel = uilabel(app.RightPanel);
            app.TimeTextAreaLabel.HorizontalAlignment = 'right';
            app.TimeTextAreaLabel.FontColor = [1 0 0];
            app.TimeTextAreaLabel.Position = [50 65 33 22];
            app.TimeTextAreaLabel.Text = 'Time';

            % Create TimeTextArea
            app.TimeTextArea = uitextarea(app.RightPanel);
            app.TimeTextArea.FontColor = [1 0 0];
            app.TimeTextArea.Position = [11 87 72 23];

            % Create EffTextArea
            app.EffTextArea = uitextarea(app.RightPanel);
            app.EffTextArea.FontColor = [1 0 0];
            app.EffTextArea.Position = [11 193 62 23];

            % Create EffTextAreaLabel
            app.EffTextAreaLabel = uilabel(app.RightPanel);
            app.EffTextAreaLabel.HorizontalAlignment = 'center';
            app.EffTextAreaLabel.FontColor = [1 0 0];
            app.EffTextAreaLabel.Position = [48 171 25 22];
            app.EffTextAreaLabel.Text = 'Eff';

            % Create EnergyTextArea
            app.EnergyTextArea = uitextarea(app.RightPanel);
            app.EnergyTextArea.HorizontalAlignment = 'center';
            app.EnergyTextArea.FontColor = [1 0 0];
            app.EnergyTextArea.Position = [91 193 70 23];

            % Create EnergyTextAreaLabel
            app.EnergyTextAreaLabel = uilabel(app.RightPanel);
            app.EnergyTextAreaLabel.HorizontalAlignment = 'right';
            app.EnergyTextAreaLabel.FontColor = [1 0 0];
            app.EnergyTextAreaLabel.Position = [118 171 43 22];
            app.EnergyTextAreaLabel.Text = 'Energy';

            % Create FmEditFieldLabel
            app.FmEditFieldLabel = uilabel(app.RightPanel);
            app.FmEditFieldLabel.HorizontalAlignment = 'right';
            app.FmEditFieldLabel.Position = [7 502 25 22];
            app.FmEditFieldLabel.Text = 'Fm';

            % Create FmEditField
            app.FmEditField = uieditfield(app.RightPanel, 'numeric');
            app.FmEditField.Position = [40 502 49 22];
            app.FmEditField.Value = 100;

            % Create FbEditFieldLabel
            app.FbEditFieldLabel = uilabel(app.RightPanel);
            app.FbEditFieldLabel.HorizontalAlignment = 'right';
            app.FbEditFieldLabel.Position = [96 502 25 22];
            app.FbEditFieldLabel.Text = 'Fb';

            % Create FbEditField
            app.FbEditField = uieditfield(app.RightPanel, 'numeric');
            app.FbEditField.Position = [127 502 49 22];
            app.FbEditField.Value = 1000;

            % Create V0EditFieldLabel
            app.V0EditFieldLabel = uilabel(app.RightPanel);
            app.V0EditFieldLabel.HorizontalAlignment = 'right';
            app.V0EditFieldLabel.Position = [7 452 25 22];
            app.V0EditFieldLabel.Text = 'V0';

            % Create V0EditField
            app.V0EditField = uieditfield(app.RightPanel, 'numeric');
            app.V0EditField.Position = [40 452 49 22];
            app.V0EditField.Value = 10;

            % Create T0EditFieldLabel
            app.T0EditFieldLabel = uilabel(app.RightPanel);
            app.T0EditFieldLabel.HorizontalAlignment = 'right';
            app.T0EditFieldLabel.Position = [96 452 25 22];
            app.T0EditFieldLabel.Text = 'T0';

            % Create T0EditField
            app.T0EditField = uieditfield(app.RightPanel, 'numeric');
            app.T0EditField.Position = [127 452 49 22];

            % Create DrivingButton
            app.DrivingButton = uibutton(app.RightPanel, 'push');
            app.DrivingButton.ButtonPushedFcn = createCallbackFcn(app, @DrivingButtonPushed, true);
            app.DrivingButton.Position = [16 412 72 24];
            app.DrivingButton.Text = 'Driving';

            % Create OptionButton
            app.OptionButton = uibutton(app.RightPanel, 'push');
            app.OptionButton.ButtonPushedFcn = createCallbackFcn(app, @OptionButtonPushed, true);
            app.OptionButton.Position = [111 412 72 24];
            app.OptionButton.Text = 'Option';

            % Create Spinner
            app.Spinner = uispinner(app.RightPanel);
            app.Spinner.Limits = [0 100];
            app.Spinner.ValueChangedFcn = createCallbackFcn(app, @SpinnerValueChanged, true);
            app.Spinner.FontColor = [1 0 0];
            app.Spinner.Position = [87 371 100 22];

            % Create Position1Label
            app.Position1Label = uilabel(app.RightPanel);
            app.Position1Label.HorizontalAlignment = 'right';
            app.Position1Label.FontColor = [1 0 0];
            app.Position1Label.Position = [20 370 55 22];
            app.Position1Label.Text = 'Position1';

            % Create SpeedTextAreaLabel
            app.SpeedTextAreaLabel = uilabel(app.RightPanel);
            app.SpeedTextAreaLabel.HorizontalAlignment = 'right';
            app.SpeedTextAreaLabel.FontColor = [1 0 0];
            app.SpeedTextAreaLabel.Position = [121 65 40 22];
            app.SpeedTextAreaLabel.Text = 'Speed';

            % Create SpeedTextArea
            app.SpeedTextArea = uitextarea(app.RightPanel);
            app.SpeedTextArea.ValueChangedFcn = createCallbackFcn(app, @SpeedTextAreaValueChanged, true);
            app.SpeedTextArea.FontColor = [1 0 0];
            app.SpeedTextArea.Position = [106 87 55 23];

            % Create Spinner_2
            app.Spinner_2 = uispinner(app.RightPanel);
            app.Spinner_2.Limits = [0 100];
            app.Spinner_2.ValueChangedFcn = createCallbackFcn(app, @Spinner_2ValueChanged, true);
            app.Spinner_2.FontColor = [0 0 1];
            app.Spinner_2.Position = [87 335 100 22];

            % Create Position2Label
            app.Position2Label = uilabel(app.RightPanel);
            app.Position2Label.HorizontalAlignment = 'right';
            app.Position2Label.FontColor = [0 0 1];
            app.Position2Label.Position = [20 334 55 22];
            app.Position2Label.Text = 'Position2';

            % Create Eff2TextArea
            app.Eff2TextArea = uitextarea(app.RightPanel);
            app.Eff2TextArea.FontColor = [0 0 1];
            app.Eff2TextArea.Position = [11 140 62 23];

            % Create Eff2TextAreaLabel
            app.Eff2TextAreaLabel = uilabel(app.RightPanel);
            app.Eff2TextAreaLabel.HorizontalAlignment = 'center';
            app.Eff2TextAreaLabel.FontColor = [0 0 1];
            app.Eff2TextAreaLabel.Position = [47 118 27 22];
            app.Eff2TextAreaLabel.Text = 'Eff2';

            % Create Energy2TextArea
            app.Energy2TextArea = uitextarea(app.RightPanel);
            app.Energy2TextArea.HorizontalAlignment = 'center';
            app.Energy2TextArea.FontColor = [0 0 1];
            app.Energy2TextArea.Position = [91 140 70 23];

            % Create Energy2TextAreaLabel
            app.Energy2TextAreaLabel = uilabel(app.RightPanel);
            app.Energy2TextAreaLabel.HorizontalAlignment = 'right';
            app.Energy2TextAreaLabel.FontColor = [0 0 1];
            app.Energy2TextAreaLabel.Position = [111 118 50 22];
            app.Energy2TextAreaLabel.Text = 'Energy2';

            % Create Time2TextAreaLabel
            app.Time2TextAreaLabel = uilabel(app.RightPanel);
            app.Time2TextAreaLabel.HorizontalAlignment = 'right';
            app.Time2TextAreaLabel.FontColor = [0 0 1];
            app.Time2TextAreaLabel.Position = [45 10 38 22];
            app.Time2TextAreaLabel.Text = 'Time2';

            % Create Time2TextArea
            app.Time2TextArea = uitextarea(app.RightPanel);
            app.Time2TextArea.FontColor = [0 0 1];
            app.Time2TextArea.Position = [11 32 72 23];

            % Create Speed2TextAreaLabel
            app.Speed2TextAreaLabel = uilabel(app.RightPanel);
            app.Speed2TextAreaLabel.HorizontalAlignment = 'right';
            app.Speed2TextAreaLabel.FontColor = [0 0 1];
            app.Speed2TextAreaLabel.Position = [114 10 47 22];
            app.Speed2TextAreaLabel.Text = 'Speed2';

            % Create Speed2TextArea
            app.Speed2TextArea = uitextarea(app.RightPanel);
            app.Speed2TextArea.FontColor = [0 0 1];
            app.Speed2TextArea.Position = [106 32 55 23];

            % Create kmhLabel
            app.kmhLabel = uilabel(app.RightPanel);
            app.kmhLabel.Position = [164 86 32 22];
            app.kmhLabel.Text = 'km/h';

            % Create kmhLabel_2
            app.kmhLabel_2 = uilabel(app.RightPanel);
            app.kmhLabel_2.Position = [164 31 32 22];
            app.kmhLabel_2.Text = 'km/h';

            % Create sLabel
            app.sLabel = uilabel(app.RightPanel);
            app.sLabel.Position = [86 88 25 22];
            app.sLabel.Text = 's';

            % Create sLabel_2
            app.sLabel_2 = uilabel(app.RightPanel);
            app.sLabel_2.Position = [87 31 25 22];
            app.sLabel_2.Text = 's';

            % Create kWsLabel
            app.kWsLabel = uilabel(app.RightPanel);
            app.kWsLabel.Position = [165 192 29 22];
            app.kWsLabel.Text = 'kWs';

            % Create kWsLabel_2
            app.kWsLabel_2 = uilabel(app.RightPanel);
            app.kWsLabel_2.Position = [164 139 29 22];
            app.kWsLabel_2.Text = 'kWs';

            % Create kWLabel
            app.kWLabel = uilabel(app.RightPanel);
            app.kWLabel.Position = [171 297 25 22];
            app.kWLabel.Text = 'kW';

            % Create kWLabel_2
            app.kWLabel_2 = uilabel(app.RightPanel);
            app.kWLabel_2.Position = [170 244 25 22];
            app.kWLabel_2.Text = 'kW';

            % Create PeTextArea
            app.PeTextArea = uitextarea(app.RightPanel);
            app.PeTextArea.FontColor = [1 0 0];
            app.PeTextArea.Position = [17 298 62 23];

            % Create PeTextAreaLabel
            app.PeTextAreaLabel = uilabel(app.RightPanel);
            app.PeTextAreaLabel.HorizontalAlignment = 'center';
            app.PeTextAreaLabel.FontColor = [1 0 0];
            app.PeTextAreaLabel.Position = [54 276 25 22];
            app.PeTextAreaLabel.Text = 'Pe';

            % Create PmTextArea_2
            app.PmTextArea_2 = uitextarea(app.RightPanel);
            app.PmTextArea_2.HorizontalAlignment = 'center';
            app.PmTextArea_2.FontColor = [1 0 0];
            app.PmTextArea_2.Position = [97 298 70 23];

            % Create PmTextArea_2Label
            app.PmTextArea_2Label = uilabel(app.RightPanel);
            app.PmTextArea_2Label.HorizontalAlignment = 'right';
            app.PmTextArea_2Label.FontColor = [1 0 0];
            app.PmTextArea_2Label.Position = [124 276 43 22];
            app.PmTextArea_2Label.Text = 'Pm';

            % Create Pe2TextArea
            app.Pe2TextArea = uitextarea(app.RightPanel);
            app.Pe2TextArea.FontColor = [0 0 1];
            app.Pe2TextArea.Position = [17 245 62 23];

            % Create Pe2TextAreaLabel
            app.Pe2TextAreaLabel = uilabel(app.RightPanel);
            app.Pe2TextAreaLabel.HorizontalAlignment = 'center';
            app.Pe2TextAreaLabel.FontColor = [0 0 1];
            app.Pe2TextAreaLabel.Position = [53 223 27 22];
            app.Pe2TextAreaLabel.Text = 'Pe2';

            % Create Pm2TextArea
            app.Pm2TextArea = uitextarea(app.RightPanel);
            app.Pm2TextArea.HorizontalAlignment = 'center';
            app.Pm2TextArea.FontColor = [0 0 1];
            app.Pm2TextArea.Position = [97 245 70 23];

            % Create Pm2TextAreaLabel
            app.Pm2TextAreaLabel = uilabel(app.RightPanel);
            app.Pm2TextAreaLabel.HorizontalAlignment = 'right';
            app.Pm2TextAreaLabel.FontColor = [0 0 1];
            app.Pm2TextAreaLabel.Position = [137 223 30 22];
            app.Pm2TextAreaLabel.Text = 'Pm2';

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = Compare_DPandNMPC

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end