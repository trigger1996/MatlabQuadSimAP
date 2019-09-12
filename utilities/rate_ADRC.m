function rate_ADRC

persistent x1_p x2_p x1_p_last x2_p_last
persistent z1_p z2_p z3_p z1_p_last z2_p_last z3_p_last

persistent x1_q x2_q x1_q_last x2_q_last
persistent z1_q z2_q z3_q z1_q_last z2_q_last z3_q_last

persistent x1_r x2_r x1_r_last x2_r_last
persistent z1_r z2_r z3_r z1_r_last z2_r_last z3_r_last

global Quad

ts = Quad.Ts;

if Quad.init==0
    % roll
        x1_p = 0;
        x2_p = 0;
        x1_p_last = 0;
        x2_p_last = 0;

        z1_p = 0;
        z2_p = 0;
        z3_p  =0;
        z1_p_last = 0;
        z2_p_last = 0;
        z3_p_last = 0;

    % pitch
        x1_q = 0;
        x2_q = 0;
        x1_q_last = 0;
        x2_q_last = 0;

        z1_q = 0;
        z2_q = 0;
        z3_q  =0;
        z1_q_last = 0;
        z2_q_last = 0;
        z3_q_last = 0;        
        
    % yaw
        x1_r = 0;
        x2_r = 0;
        x1_r_last = 0;
        x2_r_last = 0;

        z1_r = 0;
        z2_r = 0;
        z3_r = 0;
        z1_r_last = 0;
        z2_r_last = 0;
        z3_r_last = 0;        
end 
        
% roll rate ADRC
    input_p  = Quad.p_des;
    feedback_p = Quad.p;

    %ts 模块 构造过渡过程
    [x1_p_last,x2_p_last] = td3(x1_p_last,x2_p_last,input_p,Quad.r_p,ts,Quad.h0_p);
    x1_p = x1_p_last;
    x2_p = x2_p_last;
    %产生误差
    e1_p = x1_p - z1_p_last;
    e2_p = x2_p - z2_p_last;
    %产生控制量
    u0 = nlsef3(e1_p,e2_p,0.3,5,0.03);
    %减去z3
    u_p = u0 - z3_p_last*1/Quad.b0_p;

    %线性eso模块
    [z1_p_last,z2_p_last,z3_p_last] = leso3(z1_p_last,z2_p_last,z3_p_last, ...
                                            Quad.w_p, ...
                                            u_p,Quad.b0_p,feedback_p,ts);
    z1_p = z1_p_last;
    z2_p = z2_p_last;
    z3_p = z3_p_last;

% pitch rate ADRC
    input_q  = Quad.q_des;
    feedback_q = Quad.q;

    %ts 模块 构造过渡过程
    [x1_q_last,x2_q_last] = td3(x1_q_last,x2_q_last,input_q,Quad.r_q,ts,Quad.h0_q);
    x1_q = x1_q_last;
    x2_q = x2_q_last;
    %产生误差
    e1_q = x1_q - z1_q_last;
    e2_q = x2_q - z2_q_last;
    %产生控制量
    u0 = nlsef3(e1_q,e2_q,0.3,5,0.03);
    %减去z3
    u_q = u0 - z3_q_last*1/Quad.b0_q;

    %线性eso模块
    [z1_q_last,z2_q_last,z3_q_last] = leso3(z1_q_last,z2_q_last,z3_q_last, ...
                                            Quad.w_q, ...
                                            u_q,Quad.b0_q,feedback_q,ts);
    z1_q = z1_q_last;
    z2_q = z2_q_last;
    z3_q = z3_q_last;    
    
% yaw rate ADRC
    input_r  = Quad.r_des;
    feedback_r = Quad.r;

    %ts 模块 构造过渡过程
    [x1_r_last,x2_r_last] = td3(x1_r_last,x2_r_last,input_r,Quad.r_r,ts,Quad.h0_r);
    x1_r = x1_r_last;
    x2_r = x2_r_last;
    %产生误差
    e1_r = x1_r - z1_r_last;
    e2_r = x2_r - z2_r_last;
    %产生控制量
    u0 = nlsef3(e1_r,e2_r,0.3,5,0.03);
    %减去z3
    u_r = u0 - z3_r_last*1/Quad.b0_r;

    %线性eso模块
    [z1_r_last,z2_r_last,z3_r_last] = leso3(z1_r_last,z2_r_last,z3_r_last, ...
                                            Quad.w_r, ...
                                            u_r,Quad.b0_r,feedback_r,ts);
    z1_r = z1_r_last;
    z2_r = z2_r_last;
    z3_r = z3_r_last;
    
% roll
    Quad.U2 = u_p * 1.0 * 10^(-3);
    Quad.U2 = min(Quad.U2_max, max(Quad.U2_min, Quad.U2));
% pitch
    Quad.U3 = u_q * 1.0 * 10^(-3);
    Quad.U3 = min(Quad.U3_max, max(Quad.U3_min, Quad.U3));
% yaw
    Quad.U4 = u_r * 1.0 * 10^(-3);
    Quad.U4 = min(Quad.U4_max, max(Quad.U4_min, Quad.U4));
    
end

    function [x1_new,x2_new] = td3(x1_last,x2_last,input,r,h,h0)
        x1_new = x1_last+h*x2_last;
        x2_new = x2_last+h*fhan(x1_last-input,x2_last,r,h0);
    end

    function u0 = nlsef3(e1,e2,c,r,h1)
        u0 = -fhan(e1,c*e2,r,h1);
    end
    
    function [z1_new,z2_new,z3_new] = leso3(z1_last,z2_last,z3_last, ...
                                        w, ...
                                        input,b0,output,h)
        beta_01 = 3*w;
        beta_02 = 3*w^2;
        beta_03 = w^3;
        e = z1_last-output;
        z1_new = z1_last+h*(z2_last-beta_01*e);
        z2_new = z2_last + h*(z3_last-beta_02*e+b0*input);
        z3_new = z3_last + h*(-beta_03*e);
    end
    
    function fh = fhan(x1_last,x2_last,r,h0)
        d = r*h0;
        d0 = h0*d;
        x1_new = x1_last+h0*x2_last;
        a0 = sqrt(d^2+8*r*abs(x1_new));
        if abs(x1_new)>d0
            a=x2_last+(a0-d)/2*sign(x1_new);
        else
            a = x2_last+x1_new/h0;
        end
        fh = -r*sat(a,d);
    end

    function M=sat(x,delta)
        if abs(x)<=delta
            M=x/delta;
        else
            M=sign(x);
        end
    end
    
    
