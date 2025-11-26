function drawFrame(T, scale)
    O = T(1:3,4);
    X = O + scale * T(1:3,1);
    Y = O + scale * T(1:3,2);
    Z = O + scale * T(1:3,3);

    % plot3([O(1) X(1)], [O(2) X(2)], [O(3) X(3)], 'r', 'LineWidth',2);
    % plot3([O(1) Y(1)], [O(2) Y(2)], [O(3) Y(3)], 'g', 'LineWidth',2);
    % plot3([O(1) Z(1)], [O(2) Z(2)], [O(3) Z(3)], 'b', 'LineWidth',2);
end