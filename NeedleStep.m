function y = NeedleStep(x, u, h, d)
    k1 = rhs(x, u, d);
    y = x + h * rhs(x + h * k1 / 2, u, d);
end

function f = rhs(x, u, d)
    v = x(:, 4);
    a = x(:, 5);
    b = x(:, 6);
    g = x(:, 7);
    k = d.ka - d.kb * u(:, 2) .^ 2;
    f = [v .* cos(a) .* cos(b), v .* sin(a) .* cos(b), v .* sin(b), u(:, 1), k .* v .* sin(g) ./ cos(b), k .* v .* cos(g), u(:, 2)];
end
