function rotation = NeedleFrame(alpha, beta, gamma)
    tangent = [cos(alpha) * cos(beta); sin(alpha) * cos(beta); sin(beta)];
    elevation = [-cos(alpha) * sin(beta); -sin(alpha) * sin(beta); cos(beta)];
    azimuth = [-sin(alpha); cos(alpha); 0];
    local_x = cos(gamma) * elevation + sin(gamma) * azimuth;
    local_y = cross(tangent, local_x);
    rotation = [local_x, local_y, tangent];
end
