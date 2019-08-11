#include <vector>
using namespace std;
template <typename T>
class RingBuffer{
public:
    class iterator: public std::reverse_iterator<typename vector<T>::iterator>{
        private:
            typename vector<T>::iterator vecIt_;
            RingBuffer* buffer_;
        public:
            iterator(){};
            iterator(RingBuffer* buffer, typename vector<T>::iterator vecIt): buffer_(buffer), vecIt_(vecIt){};
            iterator  operator++(int) /* postfix */{
                if(vecIt_ == buffer_->data_.end()){
                    return iterator(buffer_, buffer_->data_.begin());
                }
                else{
                    vecIt_ ++;
                    return iterator(buffer_, vecIt_);
                } 
            }
            iterator operator--(int){
                if(vecIt_ == buffer_->data_.begin()){
                    return iterator(buffer_, buffer_->data_.end() - 1);
                }
                else {
                    vecIt_ --;
                    return iterator(buffer_, vecIt_);
                }
            }
            iterator operator- (int prev){
                iterator newVecIt(buffer_, vecIt_);
                for(int i=0; i<prev; i++){
                    newVecIt --;
                }
                return newVecIt;
            }
            
            T& operator* () { return *vecIt_; }
            T* operator->() {return &(*vecIt_);}
    };

    vector<T> data_;
    typename vector<T>::iterator currVecIt_;
    int size_;
    RingBuffer(int size){
        size_ = size;
        data_.reserve(size);
    }
    void push_back(T element){
        if(data_.size() < size_){
            data_.push_back(element);
            currVecIt_ = data_.end() - 1;
        }
        else{
            currVecIt_++;
            if(currVecIt_ == data_.end()){
                currVecIt_ = data_.begin();
            }
            *currVecIt_ = element;
        }
    }
    size_t size(){
        return data_.size();
    }
    iterator end(){
        return iterator(this, currVecIt_);
    }

};