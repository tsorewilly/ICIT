function ang=angleBTW(ptA, ptB, ptC, dim)
    if(~exist('dim','var'))
        dim=2;
    else
        dim=width(ptA);
    end
    if width(ptA)~=width(ptB)
        error('Use Points with The Same Dimension');
    end
    if(dim==2)
        vecCA = normzz([ptA(1)-ptC(1) ptA(2)-ptC(2)]);
        vecCB = normzz([ptB(1)-ptC(1) ptB(2)-ptC(2)]);
        res = vecCA(1)*vecCB(1) + vecCA(2)*vecCB(2);
    else
        vecCA = normzz([ptA(1)-ptC(1) ptA(2)-ptC(2) ptA(3)-ptC(3)]);
        vecCB = normzz([ptB(1)-ptC(1) ptB(2)-ptC(2) ptB(3)-ptC(3)]);
        res = vecCA(1)*vecCB(1) + vecCA(2)*vecCB(2) + vecCA(3)*vecCB(3);
    end
    %Now, we can calculate the dot product:
    
    %Finally, we derive the angle between the vectors as:
    ang = acos(res);
end
