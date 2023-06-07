function [newAngle, col, UpdatedPnt]=compNextAngle(col, twist, L, CurPnt, TarPnt, PrevAngles)
    col=[col length(col)+1];    %Add Next Link Id;
    newAngle=cosRule(CurPnt, getDefaultLinkPosition(twist(col), L(col), [PrevAngles' 0]), TarPnt);
    newAngle=CorrectValueAngSign(twist(col), L(col), TarPnt, [PrevAngles' newAngle], 0);    
    UpdatedPnt=getDefaultLinkPosition(twist(col), L(col), [PrevAngles' newAngle]);
end