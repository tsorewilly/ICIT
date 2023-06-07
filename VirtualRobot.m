function [VPtCoord, Mid]=VirtualRobot(Lengths, Initial, Final, ptName)    
    if(~exist('ptName','var'))
        ptName=' ';
    end
    %Check if All Values in Length Vector are Number
    LinkNo=length(Lengths);    
    if (all(isnumeric(Lengths))==0)
       error('Enter an integer value for number of link, not a %s',class(n))
    end  
        
    %Divide the Robot into two halves base on Link Number
    if(mod(LinkNo,2)==0)       %For Robot with Even Number of Links
        Mid=(LinkNo/2);        %MidMinus1=LinkNo/2;   MidPus1=(LinkNo/2)+2;      
    else                       %For Robot with Odd Number of Links
        Mid = (LinkNo+1)/2;    %MidMinus1=(LinkNo-1)/2;   MidPus1=(LinkNo+3)+2;       
    end
    
    %Compute Lengths of Virtual Links L1 & L2
    VL1=sum(Lengths(1:Mid)); %Add 1 if L(1)=0, that is base length 
    VL2=sum(Lengths(Mid+1:LinkNo));

    VPtCoord = getThirdPoint(Initial, Final, VL1, VL2, ptName);
end