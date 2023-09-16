function prob = SedumiToMosek_Latest(At,b,c,K)
    %Convert data in Sedumi Standard Primal form to Mosek 
    %Authors: Feng-Yi Liao & Yang Zheng
    %         SOC Lab @UC San Diego
    %Update : 04/17/2022

    %*******Important********
    %We consider the following 5 cones 
    %1. free(K.f)
    %2. non-negative(K.l)
    %3. second-order cone(K.q)
    %4. roated-second-order cone(K.r)
    %5. semidefinite(K.s)
    
    %*******Important********
    %Please make sure that the problem data for PSD variable is symmetric!!
    %Otherwise, it will generate an unexpected result


    if isfield(K,'f')
        NumOfFree = K.f;
    else
        K.f = 0;
        NumOfFree = 0;
    end 
    
    if isfield(K,'l')
        NumOfNonneg = K.l;
    else
        K.l = 0;
        NumOfNonneg = 0;
    end 
    
    if isfield(K,'q')
        NumOfQuad = length(K.q);
    else
        K.q = [];
        NumOfQuad = 0;
    end
    
    if isfield(K,'r')
        NumOfRQuad = length(K.r);
    else
        K.r = [];
        NumOfRQuad = 0;
    end
    
    if isfield(K,'s')
       NumOfPSD = length(K.s);
    else
       K.s = [];
       NumOfPSD = 0;
    end
    %[r, res] = mosekopt('symbcon echo(0)');

    %==========================================
    LenFree   = NumOfFree;
    LenNonneg = NumOfNonneg;
    LenQuad   = sum(K.q,'all');%total number of Quad variables
    LenRQuad  = sum(K.r,'all');%total number of Rotated Quad variables
    LenLinear = NumOfFree+LenNonneg+LenQuad+LenRQuad;
    steps     = K.s.*(K.s+1)./2;
    LenPSD    = sum(steps,'all'); %total number of PSD variables (only symmetric part)   
    
    
    [hei,wei] = size(At);
    m = hei; %number of constraints
    %At_Free = At(:,1:NumOfFree);
    %At_Quad = At(:,NumOfFree+1:NumOfFree+LenQuad-1);
    At_Linear = At(:,1:LenLinear);
    At_PSD    = At(:,LenLinear+1:end);
    
       
    %Second Order Cone
    if (LenQuad~=0)
        QUADIDX   = zeros(1,LenQuad);
        QUADSUB   = [1,K.q(1:end-1)]; %tricky step
        QUADSUB   = cumsum(QUADSUB);
%         QUASDCONE = repmat(res.symbcon.MSK_DOMAIN_QUADRATIC_CONE,1,NumOfQuad);
        QUASDCONE = repmat(0,1,NumOfQuad);
        start     = 1;
        Offset    = NumOfFree+LenNonneg; %% notice the difference with Rotated Second constraint
        for i = 1:NumOfQuad
            step                        = K.q(i);
            QUADIDX(start:start+step-1) = (1:K.q(i))+Offset;
            start                       = start + K.q(i);
            Offset                      = Offset + step;
        end
        prob.cones.sub    = QUADIDX;
        prob.cones.subptr = QUADSUB;
        prob.cones.type   = QUASDCONE;
    end
    
    %Rotated Second Order Cone 
    if (LenRQuad ~= 0)
        RQUADIDX   = zeros(1,LenRQuad);
        RQUADSUB   = [1,K.r(1:end-1)]; %tricky step
        RQUADSUB   = cumsum(RQUADSUB);      
        %RQUASDCONE = repmat(res.symbcon.MSK_CT_RQUAD,1,NumOfRQuad);%%
        RQUASDCONE = repmat(1,1,NumOfRQuad);
        start      = 1;
        Offset     = NumOfFree+LenNonneg+NumOfQuad; %% notice the difference with Rotated Second constraint
        for i = 1:NumOfRQuad
            step                         = K.r(i);
            RQUADIDX(start:start+step-1) = (1:K.r(i))+Offset;
            start                        = start + K.r(i);
            Offset                       = Offset + step;
        end
        if LenQuad ~=0
            prob.cones.sub    = [prob.cones.sub, RQUADIDX];
            prob.cones.subptr = [prob.cones.subptr,RQUADSUB];
            prob.cones.type   = [prob.cones.type,RQUASDCONE];   
        else
            prob.cones.sub    =  RQUADIDX;
            prob.cones.subptr = RQUADSUB;
            prob.cones.type   = RQUASDCONE;    
        end
    end
    
    
    
    % Dimensions of PSD variables
    prob.bardim = K.s;   
   
    %PSD Cone (Preparation)
    IntIdxsPSD = zeros(1,LenPSD);

    %fisrt construct the corresponding idx, row, column of each variables 
    PSDIDX     = zeros(1,LenPSD);
    symrows    = zeros(1,LenPSD);
    symcols    = zeros(1,LenPSD);
    start      = 1;
    OffSet = 0; %For InterestedIdx
    for i = 1:NumOfPSD
        step                        = K.s(i)*(K.s(i)+1)/2;
        PSDIDX(start:start+step-1)  = i;
        [IndSym,~]                  = SymmetricIndices(K.s(i),false);
        [row,col]                   = ind2sub([K.s(i),K.s(i)],IndSym);
        symrows(start:start+step-1) = row;
        symcols(start:start+step-1) = col;
        IntIdx = IndSym';
        if i>1
            OffSet = OffSet + K.s(i-1)^2;
        end
        IntIdx                         = IntIdx+OffSet;
        IntIdxsPSD(start:start+step-1) = IntIdx;
        start                          = start+step;
    end
       
    
    %Objective
    c_lin = c(1:LenLinear); %free part of c
    c_s   = c(LenLinear+1:end); %PSD part of c
    
    %Obj_linear part
    prob.c = c_lin;
    
    %upper/lower bounds for constraints and variables
    prob.blc = b;
    prob.buc = b;
    prob.blx = [-inf*ones(1,LenFree), zeros(1,LenNonneg),-inf*ones(1,LenQuad),-inf*ones(1,LenRQuad)]';
    %prob.bux = [];
    prob.bux = inf*ones(LenLinear,1);
    
    
    
    %Obj_PSD
    [r,~,v]        = find(c_s(IntIdxsPSD));
    prob.barc.subj = PSDIDX(r);
    prob.barc.subk = symrows(r);
    prob.barc.subl = symcols(r);
    prob.barc.val  = v'; 


    %Constraint
    %constraint_linear part
    %if NumOfFree ~= 0 || NumQuad ~=0
    if LenLinear ~=0
        %[r,c,v]=find(At(:,1:NumOfFree+LenQuad));
        [r,c,v] = find(At_Linear);
        prob.a  = sparse(r,c,v,m,LenLinear);
    else
        prob.a  = sparse([], [], [], m, 0); 
    end    
    
        
    %constraint_psd part
    [r,c,v]        = find(At_PSD(:,IntIdxsPSD));
    prob.bara.subk = symrows(c);
    prob.bara.subl = symcols(c);
    prob.bara.subi = r';
    prob.bara.subj = PSDIDX(c);
    prob.bara.val  = v';    
    
    
end