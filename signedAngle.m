function angle=signedAngle(Va, Vb, Origin, Vn)
   if nargin==3
       Va=Va-Origin;        Vb=Vb-Origin;       
   end
   if nargin<4
       Vn=[1 0 0];
   end
   angle= atan2(dot(cross(Vb, Va),Vn), dot(Va, Vb));
end