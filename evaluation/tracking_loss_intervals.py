"""Test-only loss interval bookkeeping, indexed by actor instead of list order."""


def close_loss_interval(events, person, now):
    for event in reversed(events):
        if event['person'] == person and 'end_sim_time_s' not in event:
            event['end_sim_time_s'] = now
            return
    raise ValueError(f'No open loss interval for {person}')
