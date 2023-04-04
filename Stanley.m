clc;	clear all;	close all;
%% Scene Construction
whiteStrip=0;	rdLen=1000;	sceneHt=20;
%% Path (To be followed) Construction
pathX(1:1.1*rdLen)=1:1:1.1*rdLen;
pathY(1:1.1*rdLen)=0;
pathY(1:rdLen/2)=-3;
pathY((rdLen/2)+1:1.1*rdLen)=3;
xx = linspace(0,40);
yy = 3*sin(-(xx/8)+177.5);
q(1:63)=0-yy(1:63);
pathY(rdLen/2-31:rdLen/2+31)=q(1:63);
%% Stanley Technique
ld=10;                      % LookAhead = 10 meters
L=2;                        % Wh Base   = 2 meters
delta=0;                    % Steering  = 0 degree
v=0;                        % Velocity  = 0 kmph
crossErr1(1:rdLen*3,1:6)=0; % CrossTrack= 0 m
k_gain(1:6)=[1 2 4 6 8 10]; % for k-gain of 1, 2, 4, 6, 8 & 10
vt(:)=[5 10 15 20];         % Vel 5, 10, 15 & 20 m/s
delta_T=0.1;
thetav(1:1.1*rdLen)=0;
thetav(1:L+2)=pi/12;        % Initial Heading  = 15 degree
vehY(1:1.1*rdLen)=0;
vehY(1:L+2)=-5;             % Veh parked at Rd Side at (x,y)=(1,210)
vehX(1:1.1*rdLen)=1:1:1.1*rdLen;
goalX=0;	goalY=0;
min=rdLen;
for vv=1:1:4
    switch (vv)
        case 1 figure('Name','Stanley : Tracking @ v=5m/s','NumberTitle','off');
        case 2 figure('Name','Stanley : Tracking @ v=10m/s','NumberTitle','off');
        case 3 figure('Name','Stanley : Tracking @ v=15m/s','NumberTitle','off');
        case 4 figure('Name','Stanley : Tracking @ v=20m/s','NumberTitle','off');
    end
    t = tiledlayout(3,2);
    v=vt(vv);
    for kk=1:1:6
        for start=L+1:1:5000 
            wp=[pathX(floor(vehX(start)-L):ceil(vehX(start))+ld)' pathY(floor(vehX(start)-L):ceil(vehX(start))+ld)']; pp=controllerPurePursuit('DesiredLinearVelocity',v,'LookaheadDistance',ld,'Waypoints',wp);
            [v,w] = pp([vehX(start) vehY(start) 0]);	s = info(pp);
            tgtPt=s.LookaheadPoint;
            goalX=tgtPt(1);			goalY=tgtPt(2);
            B_bar=goalX-vehX(start);	P_bar=goalY-vehY(start);
            P=B_bar*tan(thetav(start));	e=P_bar-P;
            alpha=atan(P_bar/B_bar)-thetav(start);
            crossErr1(start,kk)=e;
            delta=alpha+tan((k_gain(kk)*e)/v);
            vehX(start+1)=vehX(start)+((v*cos(delta+thetav(start))))*delta_T;
            vehY(start+1)=vehY(start)+((v*sin(delta+thetav(start)))*delta_T);
            thetav(start+1)=thetav(start)+(((sin(delta))/L)*delta_T);                       
            if (vehY(start)<goalY)
                if (thetav(start+1)<0)   thetav(start+1)=-1*thetav(start+1);
                end
            end
            if (vehY(start)>goalY)
                if (thetav(start+1)>0)   thetav(start+1)=-1*thetav(start+1);
                end
            end
            vehY(start:rdLen)=vehY(start+1);            
            if (vehX(start+1)+ld>rdLen)	break
            end
        end
        switch (kk)
            case 1
                t1=nexttile;
                title(t1,'Tracking @ k=1')	ylabel('y(m)'); xlabel('x(m)');
            case 2
                t2=nexttile;
                title(t2,'Tracking @ k=2')      ylabel('y(m)'); xlabel('x(m)');
            case 3
                t3=nexttile;
                title(t3,'Tracking @ k=4')      ylabel('y(m)'); xlabel('x(m)');
            case 4
                t4=nexttile;
                title(t4,'Tracking @ k=6')      ylabel('y(m)'); xlabel('x(m)');
            case 5
                t5=nexttile;
                title(t5,'Tracking @ k=8')      ylabel('y(m)'); xlabel('x(m)');
            case 6
                t6=nexttile;
                title(t6,'Tracking @ k=10')     ylabel('y(m)'); xlabel('x(m)');
        end
        %% Scene Construction
        whiteStrip=0;        yellowStrip_U=5;        yellowStrip_D=-5;
        axis([0 rdLen -sceneHt sceneHt])
        hold on
         rectangle('Position',[0,-sceneHt,rdLen,sceneHt-5],'FaceColor',[0.4660 0.6740 0.1880],'EdgeColor',[0.4660 0.6740 0.1880])
        rectangle('Position',[0,-5,rdLen,10],'FaceColor',[0 0 0])
        rectangle('Position',[0,5,rdLen,sceneHt],'FaceColor',[0.4660 0.6740 0.1880],'EdgeColor',[0.4660 0.6740 0.1880])
        rectangle('Position',[rdLen/2+20,-4,60,3],'FaceColor',[1 0 0]) %Rd Block
        plot([0 rdLen],[whiteStrip whiteStrip],'--','color','white','LineWidth',2)
        plot(pathX(:),pathY(:),'LineWidth',2)
        plot(vehX(:),vehY(:),'r','LineWidth',2);
        plot(vehX(1:length(vehX(:))-1),crossErr1(1:length(vehX(:))-1,kk),'b','LineWidth',2);
        legend('Road Separation','refPath','Stanley','Crosstrack Err');
    end
    hold off
    switch (vv)
        case 1
            figure('Name','Stanley : CrossTrack Error @ v=5m/s','NumberTitle','off');
            title('CrossTrack Error @ v=5m/s'); ylabel('CrossTrack Error (m)'); xlabel('Station (m)');
        case 2
            figure('Name','Stanley : CrossTrack Error @ v=10m/s','NumberTitle','off');
            title('CrossTrack Error @ v=10m/s'); ylabel('CrossTrack Error (m)'); xlabel('Station (m)');
        case 3
            figure('Name','Stanley : CrossTrack Error @ v=15m/s','NumberTitle','off');
            title('CrossTrack Error @ v=15m/s'); ylabel('CrossTrack Error (m)'); xlabel('Station (m)');
        case 4
            figure('Name','Stanley : CrossTrack Error @ v=20m/s','NumberTitle','off');
            title('CrossTrack Error @ v=20m/s'); ylabel('CrossTrack Error (m)'); xlabel('Station (m)');
    end
    hold on
    axis([0 1000 -0.8 0.8])
     ll=0;
    if (length(vehX(:))<length(crossErr1(:,kk)))	ll=length(vehX(:));
    else	ll=length(crossErr1(:,kk));
    end   
    plot(vehX(L+1:ll),crossErr1(L+1:ll,1:6),'LineWidth',2);
    legend('k=1','k=2','k=4','k=6','k=8','k=10');
    hold off 
end
