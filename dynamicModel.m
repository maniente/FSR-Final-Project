function [xDot, x_c,y_c, fr, fcontact, ground] = dynamicModel(t,x,param)
    % ==============================================================================================================
    %% Basic parameters
    % ==============================================================================================================
    ut          = param.ut;                     % Coulomb friction coefphicient of a ground in tangential direction
    un          = param.un;                     % Coulomb friction coefphicient of a ground in normal direction 
    ct          = param.ct;                     % Viscous friction coefphicient of a ground in tangental direction
    cn          = param.cn;                     % Viscous friction coefphicient of a ground in normal direction
    N           = param.N;                      % Number of snake robot links
    kp          = param.kp;                     % Gain for position controller
    kd          = param.kd;                     % Gain for velocity controller
    l           = param.l;                      % Radius of snake robot link length
    diameter    = param.diameter;               % Diameter of a pipeline
    diameterI   = param.diameterInfluence;      % Auxiliary variable
    m           = param.m;                      % Weight of snake robot link
    g           = param.g;                      % Gravitational acceleration
    offset      = param.offset;                 % Parameter of lateral undulation pattern
    alphaA      = param.alphaA;
    omega       = param.omega;
    delta       = param.delta;
    viscous     = param.friction;                % Auxiliary variable for phirction determination
    contact     = param.contact;                % Auxiliary variable for contact determination
    utPipe      = param.utPipe;                 % Coulomb friction coefphicient of a side wall
    ctPipe      = param.ctPipe;                 % Viscous friction coefphicient of a side wall
    Erub        = param.Erub;                   % Contact parameter
    vrub        = param.vrub;                   % Contact parameter
    umax        = param.umax;                   % Maximum snake robot link torque
    qmax        = param.qmax;                   % Maximum snake robot link angular velocity
    minLinkVel  = param.minLinkVel;             % Auxiliary parameter 
    d = diameter - 2*l;                         % Theoretical diameter for snake robot links
    I = (m*(2*l)^2)/3;                          % Moment of inertia
    % ==============================================================================================================
    %% Auxiliary matrices
    % ==============================================================================================================
    for i1=1:N-1
        for i2 = 1:(N)
            if(i1==i2)
                A(i1,i2) = 1;
            end
            if(i2==i1+1)
                A(i1,i2) = 1;
            end
        end
    end
    
    for i1=1:N-1
        for i2 = 1:(N)
            if(i1==i2)
                D(i1,i2) = 1;
            end
            if(i2==i1+1)
                D(i1,i2) = -1;
            end
        end
    end
    
    Va= A'/(D*D')*A;
    Ka = A'/(D*D')*D;
        
    for i1=1:N
        for i2 = 1:(N+1)
            if(i1==i2)
                J(i1,i2) = -1;
            end
            if(i2==i1+1)
                J(i1,i2) = 1;
            end
        end
    end
    K = zeros(N,N+1);
    for i1=1:N
        for i2 = 1:N+1
            if(i1==i2)
                K(i1,i2) = 1;
            end
        end
    end
    
    for i1=1:N
        for i2 = 1:(N-1)
            if(i1==i2)
                J2(i1,i2) = 1;
            end
            if(i1==i2+1)
                J2(i1,i2) = 1;
            end
        end
    end
    
    for i1=1:N
        for i2 = 1:(N-1)
            if(i1==i2)
                Jf(i1,i2) = 1;
            end
            if(i1==i2+1)
                Jf(i1,i2) = -1;
            end
        end
    end
    
    HH = ones(N,N);
    HH = triu(HH);
    
    J3 = -J2;
    J1 = -Jf;
    NN = J*pinv(K);
    T = abs(NN);
    
    e = ones(1,N+1)'; e(length(e)) = 0;
    k = ones(1,N)';   
    j = zeros(1,N)';
    j(N) = -1;   
    % ==============================================================================================================
    %% Kinematic model
    % ==============================================================================================================
    phi     = x(1:N);
    p       = x(N+1:N+2);
    phiDot  = x(N+3:2*N+2);
    pDot    = x(2*N+3:2*N+4);

    theta = HH*phi;
    thetaDot = HH*phiDot;

    for o1=1:N
        s_vect(o1,1) = sin(theta(o1));
        c_vect(o1,1) = cos(theta(o1));
        sgn(o1,1) = sign(theta(o1));
        dThetaqSqared(o1,1) = thetaDot(o1)^2;
    end
    
    for o1=1:N
        for o2=1:N
            if(o1==o2)
                Cm(o1,o2) = cos(theta(o2));
                Sm(o1,o2) = sin(theta(o2));
            end
        end
    end

    A = J*J';
    B = (1/N)*j;
    C = (1/N)*j';
    D = (1/N);

    DD = inv(D-C*inv(A)*B);
    AA = inv(A)+inv(A)*B*DD*C*inv(A);
    BB = -inv(A)*B*DD;
    CC = -DD*C*inv(A);

    Hinv = [J' (1/N)*e]*[AA BB;CC DD];
    
    X = Hinv*[2*l*c_vect;p(1) - (l/N)*k'*c_vect];
    Y = Hinv*[2*l*s_vect;p(2) - (l/N)*k'*s_vect];
    
    dX = -J'*AA*2*l*Sm*thetaDot - (1/N)*e*CC*2*l*Sm*thetaDot + J'*BB*pDot(1) + J'*BB*(l/N)*k'*Sm*thetaDot + (1/N)*e*DD*pDot(1) + (1/N)*e*DD*(l/N)*k'*Sm*thetaDot;
    dY =  J'*AA*2*l*Cm*thetaDot + (1/N)*e*CC*2*l*Cm*thetaDot + J'*BB*pDot(2) - J'*BB*(l/N)*k'*Cm*thetaDot + (1/N)*e*DD*pDot(2) - (1/N)*e*DD*(l/N)*k'*Cm*thetaDot;
    
    Xc = K*X + l*c_vect;
    Yc = K*Y + l*s_vect;
    
    x_c = Xc;
    y_c = Yc;

    dXc = K*dX - l*Sm*thetaDot;
    dYc = K*dY + l*Cm*thetaDot;  
    % ==============================================================================================================
    %% Friction
    % ==============================================================================================================
    % Viscous
    if(viscous==1)
        fr = -[ct*(Cm*Cm)+cn*(Sm*Sm), (ct-cn)*Sm*Cm;(ct-cn)*Sm*Cm, ct*(Sm*Sm)+cn*(Cm*Cm)]*[dXc;dYc];
    else     
    % Coulomb
        fr = -m*g*[ut*Cm, -un*Sm;ut*Sm, un*Cm]*sign([Cm, Sm;-Sm, Cm]*[dXc;dYc]);
    end
    % ==============================================================================================================
    %% Contact model
    % ==============================================================================================================
    if contact == 1
        for i1=1:N
            fcn(i1,1) = 0;
            fctBo1l(i1,1) = 0;
            fct(i1,1) = 0;
        end
        
        for i1=1:N  

            if((Yc(i1)>=(d/2)))
                fcn(i1,1) = -(sqrt((16*Erub*2*l)/(9*(1-vrub^2))))*((abs(Yc(i1))-(d/2))^(3/2));
                if(abs(dXc(i1))>minLinkVel)
                    fctBo1l(i1,1) = 1;
                    if(viscous == 1) 
                        fct(i1,1) = fctBo1l(i1)*ctPipe*sign(-dXc(i1));
                    else
                        fct(i1,1) = fctBo1l(i1)*(abs(fcn(i1))*utPipe*sign(-dXc(i1)));
                    end
                else
                    fctBo1l(i1,1) = 0;
                    fct(i1,1) = 0;
                end
            elseif((Yc(i1)<=(-d/2)))
                fcn(i1,1) = (sqrt((16*Erub*2*l)/(9*(1-vrub^2))))*((abs(Yc(i1))-(d/2))^(3/2));
                if(abs(dXc(i1))>minLinkVel)
                    fctBo1l(i1,1) = 1;
                    if(viscous == 1)
                        fct(i1,1) = fctBo1l(i1)*ctPipe*sign(-dXc(i1));
                    else
                        fct(i1,1) = fctBo1l(i1)*(abs(fcn(i1))*utPipe*sign(-dXc(i1)));
                    end
                else
                    fctBo1l(i1,1) = 0;
                    fct(i1,1) = 0;
                end
            else
                fcn(i1,1) = 0;
                fctBo1l(i1,1) = 0;
                fct(i1,1) = 0;
            end
            
        end

        fcontact = [fct;fcn]; % Tangential and Normal forces against walls
    else
        fcontact = zeros(N*2,1);
        fct = zeros(N,1);
        fcn = zeros(N,1);
        fctBo1l = zeros(N,1);
    end

    ground = fcontact + fr;
    % ==============================================================================================================
    %% Dynamic model
    % ==============================================================================================================
    for i=1:N-1
        phi_required         = alphaA*sin((omega*t+(i-1)*delta)) + offset;
        phiDot_required      = alphaA*omega*cos((omega*t+(i-1)*delta));
        phiDotDot_required   = -alphaA*omega*omega*sin((omega*t+(i-1)*delta));
        u(i,1)              = phiDotDot_required + kp*(phi_required - phi(i)) + kd*(phiDot_required - phiDot(i));
        if(u(i,1)>umax)
            u(i,1) = umax;
        elseif(u(i,1)<-umax)
            u(i,1) = -umax;
        end
    end

    M = I*eye(N) + m*l*l*Sm*Va*Sm + m*l*l*Cm*Va*Cm;
    W = m*l*l*Sm*Va*Cm - m*l*l*Cm*Va*Sm;

    phi2 = phi;
    phi2(N,1) = theta(N);
    phi2Dot = phiDot;
    phi2Dot(N,1) = thetaDot(N);

    i1M = [HH'*M*HH zeros(N,2);zeros(2,N) N*m*eye(2,2)];
    WWW = [HH'*W*diag(HH*phiDot)*HH*phiDot;zeros(2,1)];
    GGG = [-l*HH'*Sm*Ka l*HH'*Cm*Ka;-k' zeros(1,N);zeros(1,N) -k'];
    BBB = [eye(N-1,N-1);zeros(3,N-1)];

    i11 = i1M(1:N-1,1:N-1);
    i12 = i1M(1:N-1,N:N+2);
    M21 = i1M(N:N+2,1:N-1);
    M22 = i1M(N:N+2,N:N+2);
    W1  = WWW(1:N-1);
    W2  = WWW(N:N+2);
    G1  = GGG(1:N-1,1:2*N);
    G2  = GGG(N:N+2,1:2*N);

    Aq  = -inv(M22)*(W2 + G2*fr + G2*fcontact);
    Bq  = -inv(M22)*M21;    
    xDot = [phiDot;pDot;u;Aq+Bq*u];
end
    


