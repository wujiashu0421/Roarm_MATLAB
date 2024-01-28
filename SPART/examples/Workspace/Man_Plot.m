function []=Man_Plot(R0,r0,base_contour,man_contour,man_contour_end,RL,rL,n)

%--- Plot Manipulator ---%

%Base transformation
for i=1:size(base_contour,1)
    base(1:3,i) = R0*[base_contour(i,:)';0]+r0;
end

%Manipulator tarnsformation
for j=1:n-1
    for i=1:size(man_contour,1)
        Link(1:3,i,j) = RL(1:3,1:3,j)*[man_contour(i,:)';0]+rL(1:3,j);
    end
end
for i=1:size(man_contour_end,1)
        Linkn(1:3,i) = RL(1:3,1:3,n)*[man_contour_end(i,:)';0]+rL(1:3,n);
end

% Plot base and manipulator
fill(base(1,:),base(2,:),'white')
plot(base(1,:),base(2,:),'k','linewidth',2);
for j=1:n-1
    plot(Link(1,:,j),Link(2,:,j),'k','linewidth',2);
end
plot(Linkn(1,:),Linkn(2,:),'k','linewidth',2);