function reply = ReplanEvent(lib, event)
    reply = struct('action', 'rejected', 'result', [], 'brake', [], 'snapshot', event);
    d = lib.d;
    d.centers = event.centers;
    d.executed_prefix = event.executed_prefix;

    if strcmp(event.kind, 'obstacle')
        q = reshape(lib.nominal(2:end), lib.d.n, 9);
        tail = q(event.node:end, :);
        dd = d;
        dd.n = size(tail, 1);
        z = [lib.nominal(1); tail(:)];
        audit = ReplanAudit(z, dd, lib.config);
        if audit.feasible
            p = [event.start; event.centers(:)];
            check = ReplanValidate(z, dd, lib.config, 'online', p);
            if check.feasible
                reply.action = 'unchanged reference accepted';
                reply.result = struct('success', true, 'method', 'unchanged_reference', 'z', z, 'audit', audit, 'check', check, 'event', event, 'total_ms', 0);
                return
            end
        end
    end
    fast = ReplanReplan(lib, event, struct('fast_only', true));
    reply.prediction = fast;
    reply.result = fast;

    if fast.success
        reply.action = 'sensitivity accepted';
        return
    end
    reply.brake = ReplanStopState(event.start, d, lib.config);

    if ~reply.brake.feasible
        reply.action = 'braking validation rejected';
        return
    end
    stopped = event;
    stopped.start = reply.brake.stopped;
    stopped.time = event.time + reply.brake.duration;
    stopped.phase = 'stopped';
    stopped.executed_prefix = [event.executed_prefix; reply.brake.position_mm(1:end - 1, :) * d.scale / 1000];
    reply.result = ReplanReplan(lib, stopped);

    if reply.result.success
        reply.action = 'stopped-state trajectory accepted';
    else
        reply.action = 'hold: replanning validation rejected';
    end
end
