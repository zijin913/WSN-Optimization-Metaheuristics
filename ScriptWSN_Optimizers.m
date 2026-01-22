% This coACO is written and ACOveloped by Amir Parniaifard. 

clc;   close all;   clear

%% Parameters Adjusment
Input = inputdlg({'Case No. (1,2,3,4)'},'Input Parameters',[1 50]);

NRepet = 10 ;
figure_shows = 'On';
MoACOl_Name = 'WSN Coverage Optimization';
CaseNo = str2num(char(Input(1,:)));
Maxiter = 10;
switch CaseNo
    case 1
        Nsensors = 20;    RCS = 5; % Coverage Radius of each sensor
    case 2
        Nsensors = 20;    RCS = 10; % Coverage Radius of each sensor
    case 3
        Nsensors = 40;    RCS = 5; % Coverage Radius of each sensor
    case 4
        Nsensors = 40;    RCS = 10; % Coverage Radius of each sensor
end

% Inpop   = Nsensors; % Equal to number of sensors in the moACOl
M_A = 100;  N_A = M_A;    % Monotoring area M*N pixe
NVar = Nsensors*2;
lob = ones(NVar,1)' * 1  ;
upb = ones(NVar,1)' *  M_A  ;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Algorithms coACOs

for InACOnting_CoACO = 1:1
    %------Obtain coordinates of obstacles ------
    Network_Area = zeros(M_A,M_A);
    [x,y]=meshgrid(1:M_A);% filter indise circle
        i0=25; j0=75;R=10; Network_Area((x-i0).^2+(y-j0).^2<R^2) = 1;
    i0=75; j0=25;R=8; Network_Area((x-i0).^2+(y-j0).^2<R^2) = 1;
    Network_Area(74:78,61:81) = 1; Network_Area(66:86,69:74) = 1;
    Network_Area(12:28,28:32) = 1; Network_Area(18:22,22:38) = 1;
    Xo = sum(Network_Area,2);
    Yo = sum(Network_Area,1);
    l =0; m = 0;
    for i = 1:M_A
        if Xo(i) > 0
            l = l+1;
            Xobst(l,1) = i;
        end
        if Yo(i) > 0
            m = m+1;
            Yobst(m,1) = i;
        end
    end
    %-------- Infeasible pixels ---------------------------------
    Total_pixels = gridsamp([1 1;M_A M_A],M_A);
    L = 0;
    for i = 1: size(Total_pixels,1)
        pixel = Total_pixels(i,:);
        A = any(Xobst(:) == pixel(1));
        B = any(Yobst(:) == pixel(2));
        if A==1 && B==1
            L = L+1;
            Inf_pixel_no(L) = i;
        end
    end
    Inf_pixel = Total_pixels(Inf_pixel_no,:);
    Fes_pixel = Total_pixels;
    Fes_pixel(Inf_pixel_no,:) = [];

    for rep = 1:NRepet
        clc; disp(['Rep:  ',num2str(rep)]);
        %%%%%%%%%%%% PSO optimizer ---------------------------------
        tic
        PSO_opt_sensLoc= round(Particle_Swarm_Optimization(Nsensors,NVar,[lob;upb]',@Opt_FitFunc,'min',2,2,2,0.4,0.9,Maxiter));
        PSO_opt_sensLoc_record(rep,:) = PSO_opt_sensLoc;
        PSO_OptCovRate(rep) = Opt_FitFunc(PSO_opt_sensLoc)*-1;% -1 for maximization aspect of coverage rate
        time_PSO_opt(rep) = toc ;
        %%%%%%%%%%%% GWO optimizer ---------------------------------
        tic
        [OptFitness_GWO,GWO_opt_sensLoc] = GWO(Nsensors,Maxiter,lob,upb,NVar,@Opt_FitFunc);
        GWO_opt_sensLoc = round(GWO_opt_sensLoc);
        GWO_opt_sensLoc_record(rep,:) = GWO_opt_sensLoc;
        GWO_OptCovRate(rep) = OptFitness_GWO*-1; % -1 for maximization aspect of coverage rate
        time_GWO_opt(rep) = toc ;
        %%%%%%%%%%%% GA optimizer ---------------------------------
        tic
        options = optimoptions('ga','PopulationSize', Nsensors,'Generations', Maxiter);
        [GA_opt_sensLoc,OptFitness_GA] = ga(@Opt_FitFunc,NVar,[],[],[],[],lob,upb,[],options);
        GA_opt_sensLoc_record(rep,:) = GA_opt_sensLoc;
        GA_OptCovRate(rep) = OptFitness_GA*-1;% -1 for maximization aspect of coverage rate
        time_GA_opt(rep) = toc ;
        %%%%%%%%%%%% ACO optimizer ---------------------------------
        tic
        [ACO_opt_sensLoc,OptFitness_ACO] = ACO(Nsensors,Maxiter,NVar,lob,upb,@Opt_FitFunc);
        ACO_opt_sensLoc = round(ACO_opt_sensLoc);
        ACO_opt_sensLoc_record(rep,:) = ACO_opt_sensLoc;
        ACO_OptCovRate(rep) = OptFitness_ACO*-1; % -1 for maximization aspect of coverage rate
        time_ACO_opt(rep) = toc ;
        %-----------------------------------------------------------
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Results
for InACOnting_Results = 1:1
    clc;  close all;   format shortG;
    disp(['MoACOls Name:  ',MoACOl_Name]);
    disp('--------------------------------------------------------------------------------------');
    Methods= ({'PSO';'GWO';'GA';'ACO'});
    tb_Ave = [mean(PSO_OptCovRate);mean(GWO_OptCovRate);mean(GA_OptCovRate)...
        ;mean(ACO_OptCovRate)];
    tb_max = [max(PSO_OptCovRate);max(GWO_OptCovRate);max(GA_OptCovRate)...
        ;max(ACO_OptCovRate)];
    tb_min =  [min(PSO_OptCovRate);min(GWO_OptCovRate);min(GA_OptCovRate)...
        ;min(ACO_OptCovRate)];
    tb_std =  [std(PSO_OptCovRate);std(GWO_OptCovRate);std(GA_OptCovRate)...
        ;std(ACO_OptCovRate)];
    tb_Time_elapsed = [sum(time_PSO_opt,'all');sum(time_GWO_opt,'all');sum(time_GA_opt,'all')...
        ;sum(time_ACO_opt,'all')];

    Result_Table_accuracy = table(Methods,tb_Ave,tb_std,tb_max,tb_min,tb_Time_elapsed);

    Result_Table_accuracy(:,:)
    disp('--------------------------------------------------------------------------------------');

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Figures
for InACOnting_Figures = 1:1
    if strcmp(figure_shows,'On')==1
        for px = 1 : size(Total_pixels,1)
            XS =  Total_pixels(px,1);
            YS =  Total_pixels(px,2);
            plan(XS,YS) = Network_Area(px);
        end
        % -------------------------------------------------------------------------------------------------
        if CaseNo ==1
            F1 = figure;
            hAxes = gca; contourf( hAxes, plan );
            colormap( hAxes , [1 1 1; 0.3350 0.0780 0.1840] );
            title({'Network Area with Obstacles'},'FontSize',9,'fontweight','bold');
            axis equal; set(gca,'fontweight','normal','fontSize',9);
            xlim([0 M_A+1]);ylim([0 N_A+1]); xlabel('X(m)'); ylabel('Y(m)');
            box on; ax = gca;ax.BoxStyle = 'full'; grid minor; hold off
        end
        % -------------------------------------------------------------------------------------------------
        % Plot 2D map
        F2 = figure('Position', [200, 350, 1200, 350]);
        sgtitle({MoACOl_Name,['Case No. : ' num2str(CaseNo)]},...
            'fontweight','bold','FontSize',10,'FontName','Times New Roman');
        subplot(1,4,1);
        hold on;
        hAxes = gca; contourf( hAxes, plan );
        colormap( hAxes , [1 1 1; 0.6350 0.0780 0.1840] );
        [MaxCoverage,Nmax]   = max(PSO_OptCovRate) ;
        pos_record = PSO_opt_sensLoc_record(Nmax,:);
        L =1;
        for i = 1:2:NVar
            XS(L) = pos_record(i); YS(L) = pos_record(i+1);
            pos_sens = [XS',YS']; L = L+1;
        end
        % subplot(1,4,1);
        % hold on;
        % hAxes = gca; contourf(hAxes, plan);
        % colormap(hAxes, [1 1 1; 0.6350 0.0780 0.1840]);
        % [MaxCoverage, Nmax] = max(PSO_OptCovRate);
        % pos_record = PSO_opt_sensLoc_record(Nmax, :);
        % disp(['Length of pos_record: ' num2str(length(pos_record))]);
        % disp(['pos_record: ' num2str(pos_record)]);
        % L = 1;
        % for i = 1:2:min(length(pos_record)-1, NVar)
        %     XS(L) = pos_record(i);
        %     YS(L) = pos_record(i+1);
        %     pos_sens = [XS', YS'];
        %     L = L+1;
        % end

        for i=1:Nsensors
            X  = pos_sens(i,1);  Y =  pos_sens(i,2);
            plot(X,Y,'+b','LineWidth', 2); hold on
            p1 = nsidedpoly(1000, 'Center', [X Y], 'Radius', RCS); %Plot circle
            plot(p1, 'FaceColor', 'k'); %  >> 'FaceColor', ones(1,3).*rand(1,3) <<
            title({'PSO'},['%Coverage = ' num2str(MaxCoverage)],'FontSize',9,'fontweight','bold');
            axis equal
        end
        set(gca,'fontweight','normal','fontSize',9,'fontweight','normal');
        xlim([0 M_A+1]);ylim([0 N_A+1]); xlabel('X(m)'); ylabel('Y(m)');
        box on; ax = gca;ax.BoxStyle = 'full'; grid minor ; hold off

        subplot(1,4,2);
        hold on;
        hAxes = gca; contourf( hAxes, plan );
        colormap( hAxes , [1 1 1; 0.6350 0.0780 0.1840] );
        [MaxCoverage,Nmax]   = max(GWO_OptCovRate) ;
        pos_record = GWO_opt_sensLoc_record(Nmax,:);
        L =1;
        for i = 1:2:NVar
            XS(L) = pos_record(i); YS(L) = pos_record(i+1);
            pos_sens = [XS',YS']; L = L+1;
        end
        for i=1:Nsensors
            X  = pos_sens(i,1);  Y =  pos_sens(i,2);
            plot(X,Y,'+b','LineWidth', 2); hold on
            p1 = nsidedpoly(1000, 'Center', [X Y], 'Radius', RCS); %Plot circle
            plot(p1, 'FaceColor', 'k');
            title({'GWO'},['%Coverage = ' num2str(MaxCoverage)],'FontSize',9,'fontweight','bold');
            axis equal
        end
        set(gca,'fontweight','normal','fontSize',9,'fontweight','normal');
        xlim([0 M_A+1]);ylim([0 N_A+1]); xlabel('X(m)'); ylabel('Y(m)');
        box on; ax = gca;ax.BoxStyle = 'full'; grid minor ; hold off

        subplot(1,4,3);
        hold on;
        hAxes = gca; contourf( hAxes, plan );
        colormap( hAxes , [1 1 1; 0.6350 0.0780 0.1840] );
        [MaxCoverage,Nmax]   = max(GA_OptCovRate) ;
        pos_record = GA_opt_sensLoc_record(Nmax,:);
        L =1;
        for i = 1:2:NVar
            XS(L) = pos_record(i); YS(L) = pos_record(i+1);
            pos_sens = [XS',YS']; L = L+1;
        end
        for i=1:Nsensors
            X  = pos_sens(i,1);  Y =  pos_sens(i,2);
            plot(X,Y,'+b','LineWidth', 2); hold on
            p1 = nsidedpoly(1000, 'Center', [X Y], 'Radius', RCS); %Plot circle
            plot(p1, 'FaceColor', 'k');
            title({'GA'},['%Coverage = ' num2str(MaxCoverage)],'FontSize',9,'fontweight','bold');
            axis equal
        end
        set(gca,'fontweight','normal','fontSize',9,'fontweight','normal');
        xlim([0 M_A+1]);ylim([0 N_A+1]); xlabel('X(m)'); ylabel('Y(m)');
        box on; ax = gca;ax.BoxStyle = 'full'; grid minor ; hold off

        subplot(1,4,4);
        hold on;
        hAxes = gca; contourf( hAxes, plan );
        colormap( hAxes , [1 1 1; 0.6350 0.0780 0.1840] );
        [MaxCoverage,Nmax]   = max(ACO_OptCovRate) ;
        pos_record = ACO_opt_sensLoc_record(Nmax,:);
        L =1;
        for i = 1:2:NVar
            XS(L) = pos_record(i); YS(L) = pos_record(i+1);
            pos_sens = [XS',YS']; L = L+1;
        end
        for i=1:Nsensors
            X  = pos_sens(i,1);  Y =  pos_sens(i,2);
            plot(X,Y,'+b','LineWidth', 2); hold on
            p1 = nsidedpoly(1000, 'Center', [X Y], 'Radius', RCS); %Plot circle
            plot(p1, 'FaceColor', 'k');
            title({'ACO'},['%Coverage = ' num2str(MaxCoverage)],'FontSize',9,'fontweight','bold');
            axis equal
        end
        set(gca,'fontweight','normal','fontSize',9,'fontweight','normal');
        xlim([0 M_A+1]);ylim([0 N_A+1]); xlabel('X(m)'); ylabel('Y(m)');
        box on; ax = gca;ax.BoxStyle = 'full'; grid minor ; hold off
        

        % -------------------------------------------------------------------------------------------------
        F3 = figure('Position', [220, 450, 1000, 250]);
        dtaplt = [PSO_OptCovRate;GWO_OptCovRate;GA_OptCovRate;ACO_OptCovRate];
        Yb = max(dtaplt,[],'all')*1.1; Lb = min(dtaplt,[],'all')*0.9;
        hold on;
        X = 1:1:NRepet;
        plt1 = plot(X',PSO_OptCovRate,'-o','LineWidth',1,'Color',[0,0,1],...
            'MarkerSize',6,'MarkerEdgeColor','k','MarkerFaceColor',[0,0,1]);
        plt2 = plot(X',GWO_OptCovRate,'-.s','LineWidth',1,'Color',[1,0,0],...
            'MarkerSize',6,'MarkerEdgeColor','k','MarkerFaceColor',[1,0,0]);
        plt3 = plot(X',GA_OptCovRate,'--^','LineWidth',1,'Color',[0,0.5,0],...
            'MarkerSize',6,'MarkerEdgeColor','k','MarkerFaceColor',[0,0.5,0]);
        plt4 = plot(X',ACO_OptCovRate,'--*','LineWidth',1,'Color',[0,0.5,0.5],...
            'MarkerSize',6,'MarkerEdgeColor','k','MarkerFaceColor',[0,0.5,0.5]);

        set(gca,'color',[0 0 0]+0.95,'FontName','Times New Roman','FontSize',10,'XTick', 1:NRepet);
        xlabel('#Repetition'); ylabel('Coverage Rate'); ylim([Lb Yb]);
        legend([plt1 plt2 plt3 plt4],{'PSO','GWO','GA','ACO'},'Location','northeast','NumColumns',1);
        title(['Case No. : ' num2str(CaseNo)],'FontSize',10,'fontweight','bold');
        box on; hold off;

        % -------------------------------------------------------------------------------------------------
        F4 = figure('Position', [220, 300, 1000, 350]);
        subplot(1,2,1);
        hold on
        boxplot(dtaplt',1:4,'colors',[0 0.25 0.25]);
        for n = 1:4
            scatter(ones(1,NRepet)*n,dtaplt(n,:),100,'filled','^');
        end
        plt = plot((1:4),[mean(PSO_OptCovRate),mean(GWO_OptCovRate),mean(GA_OptCovRate),mean(ACO_OptCovRate)],'-h','LineWidth',0.5,'MarkerEdgeColor','k',...
            'MarkerFaceColor',[0.6350 0.0780 0.1840]);
        set(gca,'color',[0 0 0]+0.95,'FontName','Times New Roman','FontSize',10);
        legend((plt),'Mean of Data','Location','northwest');
        xlim([0 5]), xticks(1:5);  xticklabels({'PSO','GWO','GA','ACO'}); ylim([Lb Yb]);
        xlabel('Method','fontweight','bold'); ylabel('Coverage Rate','fontweight','bold');
        title(['Case No. : ' num2str(CaseNo)],'FontSize',10,'fontweight','bold');
        grid on; box on; ax = gca; ax.BoxStyle = 'full'; hold off

        subplot(1,2,2);
        hold on
        pltData = [sum(time_PSO_opt,'all');sum(time_GWO_opt,'all');sum(time_GA_opt,'all')...
            ;sum(time_ACO_opt,'all')];
        b = bar(round(pltData',3),'FaceColor',[0 .5 .5],'EdgeColor',[0.6350 0.0780 0.1840],'LineWidth',0.5);
        set(gca,'color',[1 1 1],'FontName','Times New Roman', 'FontSize',10,'fontweight','normal');
        xlabel('Method','fontweight','bold'); ylabel('Elapsed Time (s)','fontweight','bold');
        xlim([0 5]); xticks(1:4); xticklabels({'PSO','GWO','GA','ACO'});
        ylim([0 max(pltData)*1.2]);
        xtips1 = b(1).XEndPoints; ytips1 = b(1).YEndPoints;
        labels1 = string(b(1).YData); text(xtips1,ytips1,labels1,'HorizontalAlignment','center',...
            'VerticalAlignment','bottom');
        title(['Case No. : ' num2str(CaseNo)],'FontSize',10,'fontweight','bold');
        grid on; box on;ax = gca;ax.BoxStyle = 'full'; hold off
      
    end
end

%% Functions ---------------------------------------------------------
function [Percent_Cover_Rate] = Opt_FitFunc(Inset)
% Coverage optimization problem in WSN
Inset = round(Inset); % The position of sensors are integer located on pixels
[~,Nv] = size(Inset);
M = evalin('base','M_A'); % Monotoring area M*N pixel
N = evalin('base','N_A');
RCS = evalin('base','RCS'); % Coverage Radius of each sensor
Inf_pixel = evalin('base','Inf_pixel');
Fes_pixel = evalin('base','Fes_pixel');

L =1;
for i = 1:2:Nv
    XS(L) = Inset(i);
    YS(L) = Inset(i+1);
    pos_sens = [XS',YS'];
    L = L+1;
end

%%%%% check the percent coverage
total_no_pix = size(Fes_pixel,1); % If assume only feasible pixels are counted and obtstacles are not required to cover.
Total_pixels = Fes_pixel;

for px = 1:size(Total_pixels,1)
    for s = 1: size(pos_sens,1)
        sens_point = pos_sens(s,:) ;
        pixel = Total_pixels(px,:) ;
        dis2sen = norm(sens_point - pixel);
        if dis2sen <= RCS
            C = 1 ;
        else
            C = 0;
        end
        Csen(s) = C;
    end
    if sum(Csen) >= 1
        CC = 1 ;
    else
        CC = 0 ;
    end
    cov_pix(px) = CC;
end
total_pix_covered = sum(cov_pix);
Percent_Cover_Rate = (total_pix_covered / total_no_pix)*100 * -1 ; % -1 for minimization case of optimizers
% end
end




