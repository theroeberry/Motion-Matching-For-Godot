#ifndef KD_TREE_H
#define KD_TREE_H

#include "core/vector.h"

struct _DefaultDistanceFuncClass {
    float operator()(float* fa, const Vector<float>& fv) const {
        float distance = 0.f;
        for(int i = 0; i != fv.size(); ++i) {
            float temp = fa[i] - fv[i];
            distance += temp * temp;
        }
        return distance;
    }
    _FORCE_INLINE_ float distance_to_axis(float a, float b, int axis) {
        return (a - b) * (a - b);
    }
};

template <typename T, class DistanceFuncClass = _DefaultDistanceFuncClass>
class KDTree {

public:

    DistanceFuncClass distance_func_class;

private:

    int _dimension;
    class KDTreePtr {
        friend class KDTree;

        struct FeatureVector {
            float* _feature_vector;
            int _axis;
        } _feature_vector;

        KDTreePtr* _parent = nullptr;
        KDTreePtr* _left = nullptr;
        KDTreePtr* _right = nullptr;

        T _info;
        bool _visited = false;
        KDTreePtr(const T& info, float* feature_vector) : _info(info) {
            _feature_vector._feature_vector = feature_vector;
        }

        void destroy() {
            if(_feature_vector._feature_vector) delete [] _feature_vector._feature_vector;
            if(_left) _left->destroy();
            if(_right) _right->destroy();
            delete this;
        }

        KDTreePtr* search_recursive(const Vector<float>& data) {
            if(!_left && !_right) return this;
            if(!_left) return _right->search_recursive(data);
            if(!_right) return _left->search_recursive(data);
            if(data[_feature_vector._axis] <= _feature_vector._feature_vector[_feature_vector._axis]) {
                return _left->search_recursive(data);
            }
            else {
                return _right->search_recursive(data);
            }
        }
    }* _root = nullptr;

    class ComparatorClass {
    public:
        bool operator()(const KDTreePtr* lkdp, const KDTreePtr* rkdp) const {
            return lkdp->_feature_vector._feature_vector[lkdp->_feature_vector._axis] < rkdp->_feature_vector._feature_vector[rkdp->_feature_vector._axis];
        }
    };

    KDTreePtr* initialize_recursive(Vector<KDTreePtr*>& data, int left, int right, KDTreePtr* parent) {
        int mid = (left + right) / 2;
        
        int r = -1;
        float deviation = -1;
        auto temp = new float[right - left + 1];
        for(int i = 0; i != _dimension; ++i) {
            for(int j = left; j <= right; ++j) {
                temp[j - left] = (data[j]->_feature_vector._feature_vector[i]);
            }
            float t_deviation = MMMath::deviation(temp, right - left + 1);
            // Find maximum deviation
            if(t_deviation > deviation) {
                deviation = t_deviation;
                r = i;
            }
        }
        delete [] temp;

        for(int i = left; i <= right; ++i) {
            data.ptrw()[i]->_feature_vector._axis = r;
        }
        if(left != right) {
            SortArray<KDTreePtr*, ComparatorClass> sort_array;
            sort_array.sort_range(left, right, data.ptrw());
        }
        auto ptr = data[mid];
        ptr->_parent = parent;
        ptr->_left = mid > left ? initialize_recursive(data, left, mid - 1, ptr) : nullptr;
        ptr->_right = mid < right ? initialize_recursive(data, mid + 1, right, ptr) : nullptr;
        return ptr;
    }

public:

    KDTree() {};
    ~KDTree() {
        if(_root) _root->destroy();
    }

    bool initialize(const Vector<float*>& feature_vector, const Vector<T>& info, int dimension) {
        if(_root) _root->destroy();
        _root = nullptr;
        if(feature_vector.size() != info.size()) return false;
        if(dimension <= 0) return false;
        _dimension = dimension;
        Vector<KDTreePtr*> data;
        for(int i = 0; i != info.size(); ++i) {
            auto ptr = new KDTreePtr(info[i], feature_vector[i]);
            data.push_back(ptr);
        }
        
        _root = initialize_recursive(data, 0, feature_vector.size() - 1, nullptr);
        return true;
    }

    Vector<T> search(const Vector<float>& data, int n = 1) {
        _ASSERT(n > 0);
        _ASSERT(_root);
        Vector<KDTreePtr*> result;
        Vector<KDTreePtr*> visited;
        KDTreePtr* current = _root->search_recursive(data);
        current->_visited = true;
        result.push_back(current);
        visited.push_back(current);

        while(current->_parent) {
            current = current->_parent;
            while(current->_visited) {
                if(!current->_parent) goto end_of_loop;
                current = current->_parent;
            }
            current->_visited = true;
            visited.push_back(current);
            if(result.size() < n) {
                result.push_back(current);
            }
            else {
                int index = -1;
                float new_distance = distance_func_class(current->_feature_vector._feature_vector, data);
                float max_distance = -1e+20;
                for(int i = 0; i != n; ++i) {
                    float dist = distance_func_class(result[i]->_feature_vector._feature_vector, data);
                    if(dist > max_distance) {
                        max_distance = dist;
                        index = i;
                    }
                }
                if(new_distance < max_distance)
                    result.ptrw()[index] = current;
                float distance_to_axis = distance_func_class.distance_to_axis(data[current->_feature_vector._axis], current->_feature_vector._feature_vector[current->_feature_vector._axis], current->_feature_vector._axis);
                if(distance_to_axis > MIN(max_distance, new_distance)) continue;
            }

            
            if(!current->_left || !current->_right) continue;
            if(current->_left->_visited) current = current->_right->search_recursive(data);
            else if(current->_right->_visited) current = current->_left->search_recursive(data);
            current->_visited = true;
            visited.push_back(current);
            if(result.size() < n) {
                result.push_back(current);
            }
            else {
                int index = -1;
                float new_distance = distance_func_class(current->_feature_vector._feature_vector, data);
                float max_distance = new_distance;
                for(int i = 0; i != n; ++i) {
                    float dist = distance_func_class(result[i]->_feature_vector._feature_vector, data);
                    if(dist > max_distance) {
                        max_distance = dist;
                        index = i;
                    }
                }
                if(index != -1)
                    result.ptrw()[index] = current;
            }
        }
end_of_loop:
        // Reset
        for(int i = 0; i != visited.size(); ++i) {
            visited.ptrw()[i]->_visited = false;
        }
        Vector<T> info;
        for(int i = 0; i != result.size(); ++i) {
            info.push_back(result[i]->_info);
        }
        print_line(String::num_int64(visited.size()));
        visited.clear();
        return info;
    }

    KDTree(const KDTree& kdt) = delete;
    KDTree& operator=(const KDTree& kdt) = delete;
};


#endif // KD_TREE_H