/*
this class takes in a vector of zero crossings from
a sound sample and attemps to infer the fundamental frequency
to a resolution of 1 cent from its closest musical pitch.
it returns an integer value that corresponds to the
midi numbering system times 100.

examples :
return of 4487
round(4487/100) = 45 (note A second octave on midi : A2)
4487 - 45*100 = -13  (13 cent lower than A2)
*/

#include "findfun.h"


#define have_alot_of_sram false

void findfun::begin(float CON_PITCH,
                    float SAMP_FREQ,
                    int LOW_MIDI_INDEX,
                    int HIGH_MIDI_INDEX,
                    int NUM_CORR_SHIFTS)
{

    con_pitch = CON_PITCH;
    samp_freq = SAMP_FREQ;
    low_midi_index = LOW_MIDI_INDEX;
    high_midi_index = HIGH_MIDI_INDEX;
    num_corr_shifts = NUM_CORR_SHIFTS;

    max_freq = 0xFFFFFFFF;
    samp_freq_factor = max_freq / samp_freq;
#if have_alot_of_sram
    // create the test edges up front to save during execution if you have the memory
    for (auto mci = LOW_MIDI_INDEX * 100 - 100; mci < HIGH_MIDI_INDEX * 100 + 100; ++mci)
    {
        int p = ceil(max_freq / (CON_PITCH * (pow(2, ((mci - 6900.) / 1200.)))));
        int hp = ceil(p / 2);
        vector<int> e = {hp, p, p + hp};
        midi_cent_test_edges.push_back(e);
    }
#endif
}

int findfun::find_closest_period(const vector<int> &sample_edges,
                                 const vector<int> test_edges_idxs)
{

    float m_corr_accum_max_ratio = 0;
    int m_corr_accum_max_idx = 0;

    int idx = 0;

    for (auto te_idx : test_edges_idxs)
    {
#if have_alot_of_sram
        vector<int> test_edges = midi_cent_test_edges[te_idx];
#else
        int p = ceil(max_freq / (con_pitch * (pow(2, ((te_idx + (low_midi_index - 1) * 100 - 6900.) / 1200.)))));
        int hp = ceil(p / 2);
        vector<int> test_edges = {hp, p, p + hp};
#endif
        int corr_accum_max = 0;
        int right_test_edge = test_edges.back();

        vector<int> offsets;
        for (auto i = 0; i < num_corr_shifts; ++i)
        {
            offsets.push_back(ceil(right_test_edge * i / num_corr_shifts));
        }

        for (auto offset : offsets)
        {

            int left_idx = 0;
            int right_idx = 0;

            vector<int> sample_edges_shifted;

            for (auto se : sample_edges)
            {
                sample_edges_shifted.push_back(se - offset);
            }

            while (sample_edges_shifted[left_idx] <= 0)
            {
                left_idx++;
            }

            while (sample_edges_shifted[right_idx] < right_test_edge)
            {
                if (right_idx < (sample_edges_shifted.size() - 1))
                {
                    right_idx++;
                }
                else
                {
                    break;
                }
            }

            vector<int> corr_edges;
            for (auto c = left_idx; c < right_idx; ++c)
            {
                corr_edges.push_back(sample_edges_shifted[c]);
            }

            for (auto t = 0; t < test_edges.size(); ++t)
            {

                int te = test_edges[t];

                int c_idx = findfun::get_index_vector_int(corr_edges, te);

                if (c_idx >= 0)
                {
                    remove(corr_edges.begin(), corr_edges.end(), te);
                }
                else
                {
                    corr_edges.push_back(te);
                }
            }

            int corr_accum = 0;

            int num_corr_edges = corr_edges.size();
            int num_corr_edges_even = (num_corr_edges % 2) == 0;

            if (num_corr_edges)
            {

                sort(corr_edges.begin(), corr_edges.end());

                int first_sample_idx_even = (left_idx % 2) == 0;

                if (first_sample_idx_even)
                {
                    corr_accum = corr_edges[0];
                }

                if (num_corr_edges > 1)
                {

                    int ei = (first_sample_idx_even + 1);
                    for (ei; ei < corr_edges.size(); ei += 2)
                    {
                        corr_accum += (corr_edges[ei] - corr_edges[ei - 1]);
                    }

                    if (ei == corr_edges.size() - 2)
                    {
                        corr_accum += (right_test_edge - corr_edges.back());
                    }
                }
            }
            else
            {
                corr_accum = right_test_edge;
            }

            if (corr_accum > corr_accum_max)
            {
                corr_accum_max = corr_accum;
            }
        }

        float corr_accum_max_ratio = float(corr_accum_max) / float(right_test_edge);

        if (corr_accum_max_ratio > m_corr_accum_max_ratio)
        {
            m_corr_accum_max_ratio = corr_accum_max_ratio;
            m_corr_accum_max_idx = idx;
        }

        idx++;
    }

    return m_corr_accum_max_idx;
}

int findfun::find_midi_cent(const vector<int> &sample_edges)
{

    int tmci = 0; // target midi cent index
    int trmi = 0; // target relative midi index
    int trdi = 0; // target relative deca index (10 cent relative target from 150 centbin)
    int trci = 0; // target relative cent index (1 cent relative target from 15 cent bin)

    vector<int> scaled_sample_edges;

    for (int i = 1; i < sample_edges.size(); ++i)
    {
        scaled_sample_edges.push_back(samp_freq_factor * (sample_edges[i] - sample_edges[0]));
    }

    if (scaled_sample_edges.size() > 3)
    {

        vector<int> test_edges_mi_idxs;
        for (auto tp = 100; tp < (high_midi_index - low_midi_index) * 100 + 100; tp += 100)
        {
            test_edges_mi_idxs.push_back(tp);
        }
        trmi = findfun::find_closest_period(scaled_sample_edges, test_edges_mi_idxs);
        tmci = trmi * 100 + 100;

        vector<int> test_edges_mdi_idxs;
        for (auto tp = tmci - 100; tp < tmci + 100; tp += 10)
        {
            test_edges_mdi_idxs.push_back(tp);
        }
        trdi = findfun::find_closest_period(scaled_sample_edges, test_edges_mdi_idxs);
        tmci += trdi * 10 - 100;

        vector<int> test_edges_mci_idxs;
        for (auto tp = tmci - 10; tp < tmci + 10; tp++)
        {
            test_edges_mci_idxs.push_back(tp);
        }
        trci = findfun::find_closest_period(scaled_sample_edges, test_edges_mci_idxs);
        tmci += trci - 10;

        tmci += low_midi_index * 100 - 100;
    }

    return tmci;
}

int findfun::get_index_vector_int(vector<int> v, int K)
{
    auto it = find(v.begin(), v.end(), K);

    if (it != v.end())
    {
        int index = it - v.begin();
        return index;
    }
    else
    {
        return -1;
    }
}

