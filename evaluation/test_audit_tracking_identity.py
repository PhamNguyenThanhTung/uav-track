"""Regression checks for audit denominators, causal labels and concurrent loss."""
import unittest
import numpy as np
from audit_tracking_identity import score_window, loss_intervals
from tracking_loss_intervals import close_loss_interval


class AuditTests(unittest.TestCase):
    def test_wrong_assignment_unknown_and_unanchored_are_distinct(self):
        data=dict(frame=np.array([1,2,3,4,5]),times=np.array([40.,50.,51.,52.,53.]),
                  gt=np.array(['person_a','person_b','person_a','person_b','person_b']),
                  gt_fraction=np.ones(5),overlap=np.array([0,0,0,.8,0]),confidence=np.ones(5))
        # ID 9 has no clear observation until frame 5: cannot use future truth.
        result=score_window(data,{0:'1',1:'1',2:None,3:'9',4:'9'})
        self.assertEqual(result['all_classified']['counts'],
                         dict(eligible=4,correct=1,false=1,unknown=1,unanchored=1))
        self.assertEqual(result['all_classified']['assigned_coverage'],.75)
        self.assertEqual(result['all_classified']['correct_rate_among_anchored_proxy'],.5)
        self.assertEqual(result['all_classified']['correct_coverage_proxy'],.25)

    def test_one_id_switch_does_not_count_unknown_as_a_new_id(self):
        data=dict(frame=np.arange(1,5),times=np.array([50.,50.2,50.4,50.6]),
                  gt=np.array(['person_a']*4),gt_fraction=np.ones(4),overlap=np.zeros(4),confidence=np.ones(4))
        result=score_window(data,{0:'1',1:None,2:'2',3:'2'})
        self.assertEqual(result['id_switch_observations'],1)
        self.assertEqual(result['extra_ids_total'],1)
        self.assertEqual(result['clear_view_id_changes'],1)

    def test_concurrent_losses_close_the_correct_actor(self):
        events=[dict(person='person_a',start_sim_time_s=1),dict(person='person_b',start_sim_time_s=2)]
        close_loss_interval(events,'person_a',3)
        close_loss_interval(events,'person_b',5)
        self.assertEqual([e['end_sim_time_s'] for e in events],[3,5])
        with self.assertRaises(ValueError):
            close_loss_interval(events,'person_a',6)

    def test_reconstruct_losses_and_keep_open_tail(self):
        frames=[]
        for time,a,b in [(1,False,True),(2,False,False),(3,True,False),(4,True,True),(5,False,True)]:
            row={'sim_time_s':str(time)}
            for suffix,tracked in zip('abc',(a,b,True)):
                row.update({f'detected_{suffix}':'1',f'visible_{suffix}_px':'200',f'tracked_{suffix}':str(int(tracked))})
            frames.append(row)
        events=loss_intervals(frames,'tracking')
        self.assertEqual(events,[dict(person='person_a',start_sim_time_s=1.,end_sim_time_s=3.),
            dict(person='person_b',start_sim_time_s=2.,end_sim_time_s=4.),dict(person='person_a',start_sim_time_s=5.)])
        self.assertEqual(loss_intervals(frames,'detection'),[])


if __name__=='__main__':
    unittest.main()
