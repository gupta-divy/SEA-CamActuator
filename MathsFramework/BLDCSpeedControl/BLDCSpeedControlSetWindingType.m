function BLDCSpeedControlSetWindingType(mdlname, type)
% Code to set winding type for BLDCSpeedControl

% Copyright 2022-2023 The MathWorks, Inc.

    if strcmp(type, 'Wye-wound')
         if strcmp(get_param([mdlname '/BLDC'],'winding_type'), 'ee.enum.statorconnection.delta') || strcmp(get_param([mdlname '/BLDC'],'winding_type'), '2')
            set_param([mdlname '/BLDC'], 'winding_type', 'ee.enum.statorconnection.wye');    
         else 
             % Do nothing
         end
     else % Delta-wound
        if strcmp(get_param([mdlname '/BLDC'],'winding_type'), 'ee.enum.statorconnection.wye') || strcmp(get_param([mdlname '/BLDC'],'winding_type'), '1')
            set_param([mdlname '/BLDC'], 'winding_type', 'ee.enum.statorconnection.delta');
        else 
            % Do nothing
        end 
    end
end
