

#ifndef LINEPLANNING_COMBINATORICS_H
#define LINEPLANNING_COMBINATORICS_H

#include <vector>

namespace Combinatorics {

    using std::vector;
    using std::pair;

    /*bool choice_vector_atmost_inc(vector<unsigned int> &vec, unsigned int k)
    {

    }*/

    /*template <class It>
    bool choice_vector_exact_inc(It begin, It end, unsigned int k)
    {
        if (k == 0)
            return true;
        unsigned int first = *begin;
        auto begin_next = begin;
        begin_next++;
        bool f = choice_vector_exact_inc(begin_next, end, k-first);
        if (!f)
            return false;
        if (first == 0)
        {
            *begin = k;
            begin++;
            while (begin != end)
            {
                *begin = 0;
                begin++;
            }
            return true;
        }
        else if (begin_next != end)
        {
            first--;
            *begin = first;
            *begin_next = (*begin_next)+1;
            return false;
        }
        else
        {
            return true;
        }
    }*/

    template <class It>
    bool choice_vector_exact_inc(It begin, It end, unsigned int k)
    {
        auto it = begin;
        for (; it != end; it++)
        {
            if (*it > 0)
            {
                break;
            }
        }
        auto next = it;
        next++;
        auto v = *it;
        if (next != end)
        {
            *begin += v-1;
            *it -= v;
            (*next)++;
            return false;
        }
        else
        {
            *it = 0;
            *begin = k;
            return true;
        }
    }

    template <class Opt>
    class SequenceOfChoices {

        struct Stage{
            vector<Opt> options;
            unsigned int k;
        };

        vector<Stage> stages;

    public:
        SequenceOfChoices(){

        }

        SequenceOfChoices& choose(vector<Opt> &&options, unsigned int k)
        {
            Stage s;
            s.options = std::move(options);
            s.k = k;
            stages.push_back(std::move(s));
            return *this;
        }

        class iterator {
            friend SequenceOfChoices;
            const SequenceOfChoices *ptr;
            vector<vector<unsigned int>> state;
            bool expired = false;

            bool inc(unsigned int stage)
            {
                vector<unsigned int> &vec = state[stage];
                auto k = ptr->stages[stage].k;
                bool carry = choice_vector_exact_inc(vec.begin(), vec.end(), k);
                return carry;
            }
        public:

            iterator(const SequenceOfChoices &soc) : ptr(&soc)
            {
                for (unsigned int i = 0; i < ptr->stages.size(); i++)
                {
                    vector<unsigned int> si(ptr->stages[i].options.size(), 0);
                    si[0] = ptr->stages[i].k;
                    state.push_back(si);
                }
            }

            void operator++(){
                bool carry = true;
                for (unsigned int i = 0; i < ptr->stages.size(); i++)
                {
                    if (carry)
                    {
                        carry = inc(i);
                    }
                    else
                    {
                        break;
                    }
                }
                if (carry)
                    expired = true;
            }

            bool operator==(const iterator& other) const{
                if (expired != other.expired)
                    return false;
                return state == other.state;
            }

            vector<vector<pair<Opt, unsigned int>>> operator*() const{
                vector<vector<pair<Opt, unsigned int>>> r(ptr->stages.size(), vector<pair<Opt, unsigned int>>{});
                for (unsigned int i = 0; i < ptr->stages.size(); i++)
                {
                    auto &vec = r[i];
                    for (unsigned int j = 0; j < state[i].size(); j++)
                    {
                        if (state[i][j] > 0)
                        {
                            vec.push_back(std::make_pair(ptr->stages[i].options[j], state[i][j]));
                        }
                    }
                }
                return r;
            }
        };

        iterator begin() const{
            return iterator(*this);
        }

        iterator end() const{
            auto it = iterator(*this);
            it.expired = true;
            return it;
        }
    };

}

#endif //LINEPLANNING_COMBINATORICS_H
