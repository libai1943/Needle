function ids = ReplanLocalObstacles(centers, radii, box)
    closest = max(box(1, :)', min(box(2, :)', centers));
    ids = find(sum((centers - closest) .^ 2, 1) <= radii .^ 2);
end
