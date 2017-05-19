function [controller,input] = PID_step(reference, measure, controller)
    bi = controller.Dt*controller.Ki;
    ad = controller.Tf/(controller.Tf+controller.Dt);
    bd = controller.Kd/(controller.Tf+controller.Dt);
        
    P = controller.Kp*(reference-measure);
    D = ad*controller.D +bd*((reference-measure)-controller.Old_Error);
    
    % D Filter or anti-windup
    if (D < controller.Min_Input)
        D = controller.Min_Input;
    else if (D > controller.Max_Input)
            D = controller.Max_Input;
        end
    end
    
    % Anti-windup
    if (controller.Old_Input > controller.Max_Input)
        I = controller.Old_Input;
    else if (controller.Old_Input < -controller.Max_Input)
            I = controller.Old_Input;
        else
            I = controller.I + bi*(reference-measure);
        end
    end
    
    input = P + D + I;
    
    if (input < controller.Min_Input)
        input = controller.Min_Input;
    else if (input > controller.Max_Input)
            input = controller.Max_Input;
        end
    end
    
    controller.P = P;
    controller.I = I;
    controller.D = D;
    controller.Old_Error = (reference-measure);
    controller.Old_Output = measure;
    controller.Old_Input = input;
end
