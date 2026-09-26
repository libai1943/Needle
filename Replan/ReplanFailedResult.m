function s = ReplanFailedResult(d, c, reason)
    p = [zeros(9, 1); d.centers(:)];
    s = struct('success', false, 'cost', NaN, 'total_seconds', 0, 'status', reason, 'z', nan(1 + 9 * d.n, 1));
    s.check = ReplanValidate(s.z, d, c, 'offline', p);
end
