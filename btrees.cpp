#include <algorithm>
#include <array>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <cstdio>
#include <chrono>
#include <vector>
#include <functional>

/*
 * http://www.cburch.com/cs/340/reading/btree/index.html
 * https://www.cs.usfca.edu/~galles/visualization/BPlusTree.html
*/

template <class Key_Type>
struct ref_node;

template <class Key_Type>
struct leaf_node;

template <class Key_Type>
struct node_visitor
{
    virtual ~node_visitor();
    virtual bool visit(ref_node<Key_Type> & node, const std::size_t depth) = 0;
    virtual bool visit(leaf_node<Key_Type> & node, const std::size_t depth) = 0;
};

template <class K>
node_visitor<K>::~node_visitor() {}


template <class T>
class node
{
    public:

    using key_type = T;

    virtual ~node() = 0;

    node(key_type * begin, key_type * end):
        keys(begin),
        keys_end_(end),
        keys_size_max_(std::distance(begin, end)),
        split_start_index_(std::ceil(keys_size_max_ / 2.0))
    {}

    std::size_t keys_size() const
    {
        return this->keys_size_;
    }

    std::size_t keys_size_max() const
    {
        return this->keys_size_max_;
    }

    std::size_t split_start_index() const
    {
        return this->split_start_index_;
    }

    key_type * keys_begin()
    {
        return this->keys;
    }

    key_type * keys_end()
    {
        return this->keys_begin() + this->keys_size_;
    }

    key_type keys_min() const
    {
        return this->keys[0];
    }

    key_type keys_max() const
    {
        return this->keys[this->keys_size() - 1];
    }

    key_type keys_pop_min()
    {
        const key_type key = this->keys_min();
        std::move(this->keys_begin() + 1, this->keys_end(), this->keys_begin());
        --this->keys_size_;
        return key;
    }

    key_type keys_pop_max()
    {
        --this->keys_size_;
        return this->keys[this->keys_size()];
    }

    key_type * keys_lower_bound(key_type * first,
                                const key_type key)
    {
        auto iter = this->keys_end();
        for (auto i = first; i != this->keys_end(); ++i)
        {
            if (this->compare(key,*i))
            {
                iter = i;
                break;
            }
        }

        return iter;
    }

    key_type * keys_lower_bound(const key_type key)
    {
        return this->keys_lower_bound(this->keys_begin(), key);
    }

    std::size_t vacancy_insert(const key_type key)
    {
        auto key_insertion_point = this->keys_lower_bound(key);
        std::size_t key_insertion_index = std::distance(this->keys_begin(), key_insertion_point);

        const std::size_t move_count = std::distance(key_insertion_point, this->keys_end());
        std::move_backward(key_insertion_point, this->keys_end(), key_insertion_point + move_count + 1);
        *key_insertion_point = key;
        ++this->keys_size_;

        return key_insertion_index;
    }

    virtual std::pair<key_type, std::unique_ptr<node<T>>> insert(const key_type key) = 0;
    virtual bool preorder_visit(node_visitor<key_type> & visitor, const std::size_t depth) = 0;
    virtual bool postorder_visit(node_visitor<key_type> & visitor, const std::size_t depth) = 0;

    protected:

    std::less<key_type> compare;
    std::size_t keys_size_ = 0;

    private:

    key_type * keys;
    key_type * keys_end_;
    std::size_t keys_size_max_ = 0;
    std::size_t split_start_index_ = 0;
};

template <class T>
node<T>::~node() = default;


template <class Key_Type>
struct ref_node: public node<Key_Type>
{
    using node_type = node<Key_Type>;
    using ref_type = std::unique_ptr<node_type>;
    using typename node_type::key_type;

    ref_node(key_type * keys, key_type * keys_end,
             ref_type * refs, ref_type * refs_end):
        node_type(keys, keys_end),
        refs_(refs),
        refs_end_(refs_end),
        refs_size_max_(std::distance(refs, refs_end))
    {}

    virtual ~ref_node();

    std::size_t refs_size() const
    {
        return this->refs_size_;
    }

    std::size_t refs_size_max() const
    {
        return this->refs_size_max_;
    }

    ref_type * refs_begin()
    {
        return this->refs_;
    }

    ref_type * refs_end()
    {
        return this->refs_ + this->refs_size();
    }

    ref_type * refs_last()
    {
        return this->refs_ + (this->refs_size() - 1);
    }

    std::unique_ptr<node_type> & refs_min()
    {
        return this->refs_[0];
    }

    std::unique_ptr<node_type> & refs_max()
    {
        return this->refs_[this->refs_size() - 1];
    }

    virtual std::unique_ptr<ref_node> make_ref_node() = 0;

    std::unique_ptr<ref_node> split()
    {
        auto new_node = this->make_ref_node();

        std::move(this->keys_begin() + this->split_start_index(), this->keys_end(), new_node->keys_begin());
        std::move(this->refs_begin() + this->split_start_index() + 1, this->refs_end(), new_node->refs_begin());

        new_node->keys_size_ = std::distance(this->keys_begin() + this->split_start_index(), this->keys_end());
        this->keys_size_ -= new_node->keys_size();

        new_node->refs_size_ = std::distance(this->refs_begin() + this->split_start_index() + 1, this->refs_end());
        this->refs_size_ -= new_node->refs_size();

        return new_node;
    }

    void vacancy_insert(ref_type * insertion_point,
                        std::unique_ptr<node_type> && node)
    {
        const std::size_t move_count = std::distance(insertion_point, this->refs_end());
        std::move_backward(insertion_point, this->refs_end(), insertion_point + move_count + 1);
        *insertion_point = std::move(node);
        ++this->refs_size_;
    }

    void vacancy_insert(const key_type key,
                        std::unique_ptr<node_type> && node)
    {
        const std::size_t key_insertion_index = node_type::vacancy_insert(key);
        this->vacancy_insert(this->refs_begin() + key_insertion_index + 1, std::move(node));
    }

    void vacancy_insert(std::unique_ptr<node_type> && node)
    {
        ref_type * ref_insertion_point =
            this->refs_begin() + std::distance(this->keys_begin(), this->keys_lower_bound(node->keys_max()));

        this->vacancy_insert(ref_insertion_point, std::move(node));
    }

    bool preorder_visit(node_visitor<key_type> & visitor, const std::size_t depth) override
    {
        {
            const bool ret = visitor.visit(*this, depth);
            if (!ret)
            {
                return ret;
            }
        }

        const std::size_t next_depth = depth + 1;
        for (auto i = this->refs_begin(); i != this->refs_end(); ++i)
        {
            const bool ret = (*i)->preorder_visit(visitor, next_depth);
            if (!ret)
            {
                return ret;
            }
        }

        return true;
    }

    bool postorder_visit(node_visitor<key_type> & visitor, const std::size_t depth) override
    {
        const std::size_t next_depth = depth + 1;
        for (auto i = this->refs_begin(); i != this->refs_end(); ++i)
        {
            const bool ret = (*i)->postorder_visit(visitor, next_depth);
            if (!ret)
            {
                return ret;
            }
        }

        return visitor.visit(*this, depth);
    }

    void refs_push_front(std::unique_ptr<node_type> && ref)
    {
        const std::size_t move_count = this->refs_size_;
        std::move_backward(this->refs_begin(), this->refs_end(), this->refs_begin() + move_count + 1);
        this->refs_min() = std::move(ref);
        ++this->refs_size_;
    }

    std::unique_ptr<node_type> refs_pop_max()
    {
        std::unique_ptr<node_type> node = std::move(this->refs_max());
        --this->refs_size_;
        return node;
    }

    std::pair<key_type, std::unique_ptr<node_type>> insert(const key_type key)
    {
        ref_type * child_node = this->refs_last();

        if (this->compare(key,this->keys_min()))
        {
            child_node = this->refs_begin();
        }
        else if (!(this->compare(key,this->keys_max())))
        {
            child_node = this->refs_last();
        }
        else
        {
            child_node =
                this->refs_begin() + std::distance(this->keys_begin(),
                                                   this->keys_lower_bound(this->keys_begin() + 1, key));
        }

        auto child_node_ret = (*child_node)->insert(key);

        if (!child_node_ret.second)
        {
            // Child node did not split
            return std::make_pair(0, nullptr);
        }

        // Child node split, so incorporate the new key, and new node.

        if (this->keys_size() < this->keys_size_max())
        {
            // This node has enough room to store the new key and node
            vacancy_insert(child_node_ret.first, std::move(child_node_ret.second));
            return std::make_pair(0, nullptr);
        }

        // This node does not have enough room to store the new key,
        // so split it.

        auto new_node = this->split();
        if (this->compare(child_node_ret.first, this->keys_max()))
        {
            /*
             * Insert child_node_ret.first and child_node_ret.second into this
             * Move this->refs_max to new_node
             * Promote this->keys_max()
             */
            const key_type promoted_key = this->keys_pop_max();
            this->vacancy_insert(child_node_ret.first, std::move(child_node_ret.second));
            new_node->refs_push_front(std::move(this->refs_pop_max()));
            return std::make_pair(promoted_key, std::move(new_node));
        }
        else if (!(this->compare(child_node_ret.first, this->keys_max())) &&
                 this->compare(child_node_ret.first, new_node->keys_min()))
        {
            /*
             * Insert child_node_ret.second into new_node
             * Promote child_node_ret.first
             */
            new_node->refs_push_front(std::move(child_node_ret.second));
            return std::make_pair(child_node_ret.first, std::move(new_node));
        }
        else
        {
            /*
             * Insert child_node_ret.first and child_node_ret.second into new_node
             * Promote new_node->keys_min()
             */
            const key_type promoted_key = new_node->keys_pop_min();
            new_node->vacancy_insert(child_node_ret.first, std::move(child_node_ret.second));
            return std::make_pair(promoted_key, std::move(new_node));
        }
    }

    protected:

    std::size_t refs_size_ = 0;

    private:

    ref_type * refs_;
    ref_type * refs_end_;

    std::size_t refs_size_max_ = 0;
};

template <class K>
ref_node<K>::~ref_node() = default;


template <class Key_Type, std::size_t Max_Refs>
class array_ref_node: public ref_node<Key_Type>
{
    public:

    using ref_node_type = ref_node<Key_Type>;
    using typename ref_node_type::key_type;
    using typename ref_node_type::ref_type;
    using typename ref_node_type::node_type;

    array_ref_node():
        ref_node_type(this->keys_.data(), this->keys_.data() + this->keys_.size(),
                      this->refs_.data(), this->refs_.data() + this->refs_.size())
    {}

    array_ref_node(std::unique_ptr<node_type> && lower_ref,
                   const key_type key,
                   std::unique_ptr<node_type> && upper_ref):
        ref_node_type(this->keys_.data(), this->keys_.data() + this->keys_.size(),
                      this->refs_.data(), this->refs_.data() + this->refs_.size())
    {
        this->keys_[0] = key;
        this->keys_size_ = 1;

        this->refs_[0].swap(lower_ref);
        this->refs_[1].swap(upper_ref);
        this->refs_size_ = 2;
    }

    std::unique_ptr<ref_node_type> make_ref_node() override
    {
        return std::make_unique<array_ref_node>();
    }

    private:

    std::array<key_type, Max_Refs - 1> keys_;
    std::array<ref_type, Max_Refs> refs_;
};


template <class Key_Type>
struct leaf_node: public node<Key_Type>
{
    using node_type = node<Key_Type>;
    using typename node_type::key_type;

    leaf_node(key_type * begin,
              key_type * end):
        node_type(begin, end)
    {}

    virtual ~leaf_node();

    void vacancy_insert(const key_type key)
    {
        node_type::vacancy_insert(key);
    }

    std::pair<key_type, std::unique_ptr<node_type>> insert(const key_type key) override
    {
        if (this->keys_size() < this->keys_size_max())
        {
            node_type::vacancy_insert(key);
            return std::make_pair(0, nullptr);
        }

        // This node is full so split it
        auto ret = split();
        if (this->compare(key, ret->keys_min()))
        {
            // The key needs to be inserted into this node
            node_type::vacancy_insert(key);
        }
        else
        {
            // The key needs to be inserted into the new node
            ret->vacancy_insert(key);
        }

        return std::make_pair(ret->keys_min(), std::move(ret));
    }

    bool postorder_visit(node_visitor<key_type> & visitor, const std::size_t depth) override
    {
        return visitor.visit(*this, depth);
    }

    bool preorder_visit(node_visitor<key_type> & visitor, const std::size_t depth) override
    {
        return visitor.visit(*this, depth);
    }

    std::unique_ptr<leaf_node> split()
    {
        auto new_node = this->make_leaf_node();

        std::move(this->keys_begin() + this->split_start_index(), this->keys_end(), new_node->keys_begin());
        new_node->keys_size_ = std::distance(this->keys_begin() + this->split_start_index(), this->keys_end());
        this->keys_size_ -= new_node->keys_size();

        return new_node;
    }

    virtual std::unique_ptr<leaf_node> make_leaf_node() = 0;
};

template <class K>
leaf_node<K>::~leaf_node() = default;


template <class Key_Type, std::size_t Max_Keys>
struct array_leaf_node: public leaf_node<Key_Type>
{
    using leaf_node_type = leaf_node<Key_Type>;
    using typename leaf_node_type::key_type;

    array_leaf_node():
        leaf_node_type(this->keys_.data(), this->keys_.data() + this->keys_.size())
    {}

    std::unique_ptr<leaf_node_type> make_leaf_node() override
    {
        return std::make_unique<array_leaf_node>();
    }


    std::array<key_type, Max_Keys> keys_;
};

template <class Key_Type, std::size_t Order>
class bplus_tree
{
    public:

    using key_type = Key_Type;

    static constexpr std::size_t refs_size_max = Order;
    static constexpr std::size_t keys_size_max = refs_size_max - 1;


    bplus_tree():
        root(std::make_unique<array_leaf_node<key_type, keys_size_max>>())
    {}

    void insert(const key_type key)
    {
        auto ret = this->root->insert(key);
        if (!ret.second)
        {
            // The root node did not split, so there's nothing to do
            return;
        }

        // The root node split, so make a new root node
        std::unique_ptr<node<int>> new_root =
            std::make_unique<array_ref_node<key_type, refs_size_max>>(std::move(this->root),
                                                                      ret.first,
                                                                      std::move(ret.second));
        this->root.swap(new_root);
    }

    bool preorder_visit(node_visitor<key_type> & visitor)
    {
        return this->root->preorder_visit(visitor, 0);
    }

    bool postorder_visit(node_visitor<key_type> & visitor)
    {
        return this->root->postorder_visit(visitor, 0);
    }

    private:

    std::unique_ptr<node<int>> root;
};

struct checking_visitor: public node_visitor<int>
{
    std::size_t leaf_depth_ = 0;
    std::size_t * leaf_depth = nullptr;

    std::vector<int> keys;

    bool visit(ref_node<int> & node, const std::size_t depth) override
    {
        if (node.keys_size() + 1 != node.refs_size())
        {
            printf("key to ref ratio is bad\n");
            return false;
        }

        auto prev_child = node.refs_begin();
        if (!(*prev_child))
        {
            printf("child node is null\n");
            return false;
        }

        if ((*prev_child)->keys_max() >= node.keys_min())
        {
            printf("child node keys_max >= node.keys_min\n");
            return false;
        }

        for (auto i = node.refs_begin() + 1; i != node.refs_end(); ++i)
        {
            if (!*i)
            {
                printf("child node is null\n");
                return false;
            }

            if ((*prev_child)->keys_max() >= (*i)->keys_min())
            {
                printf("prev child max key is >= child min key");
                return false;
            }

            prev_child = i;
        }

        if (node.refs_max()->keys_min() < node.keys_max())
        {
            printf("refs_max.keys_min < node.keys_max\n");
            return false;
        }

        int prev_key = node.keys_min();
        for (auto i = node.keys_begin() + 1; i != node.keys_end(); ++i)
        {
            if (prev_key >= *i)
            {
                printf("previous key is >= current key\n");
                return false;
            }

            const std::size_t key_index = std::distance(node.keys_begin(), i);
            auto lower_ref_iter = node.refs_begin() + key_index;
            std::unique_ptr<::node<int>> & lower_ref = *lower_ref_iter;
            if (!lower_ref)
            {
                printf("lower ref is null\n");
                return false;
            }

            if (lower_ref->keys_min() >= *i || lower_ref->keys_max() >= *i)
            {
                printf("lower ref min key is >= current key or lower ref max key is >= current key\n");
                return false;
            }
            else if (lower_ref->keys_min() < *i && lower_ref->keys_max() >= *i)
            {
                printf("current key is between lower ref min and max keys\n");
                return false;
            }

            prev_key = *i;
        }

        return true;
    }

    bool visit(leaf_node<int> & node, const std::size_t depth)
    {
        if (leaf_depth && *leaf_depth != depth)
        {
            printf("leaf depth missmatch\n");
            return false;
        }
        else if (!leaf_depth)
        {
            // Leaf depth has not been set yet
            leaf_depth_ = depth;
            leaf_depth = &leaf_depth_;
        }

        int prev_key = node.keys_min();
        keys.push_back(prev_key);
        for (auto i = node.keys_begin() + 1; i != node.keys_end(); ++i)
        {
            keys.push_back(*i);
            if (prev_key >= *i)
            {
                printf("prev key is >= current key\n");
                return false;
            }

            prev_key = *i;
        }

        return true;
    }
};

struct printing_visitor: public node_visitor<int>
{
    bool visit(ref_node<int> & node, const std::size_t depth) override
    {
        std::cout << "ref node (" << &node << "): ";
        for (auto i = node.keys_begin(); i != node.keys_end(); ++i)
        {
            const std::size_t key_index = std::distance(node.keys_begin(), i);
            if (key_index == 0)
            {
                std::cout
                    << '(' << static_cast<void*>(node.refs_begin()[0].get()) << ')'
                    << " <- "
                    << *i
                    << " -> "
                    << '(' << static_cast<void*>(node.refs_begin()[1].get()) << ')'
                    << ' ';
            }
            else
            {
                std::cout
                    << "<- "
                    << *i
                    << " -> "
                    << '(' << static_cast<void*>(node.refs_begin()[key_index + 1].get()) << ')'
                    << ' ';
            }
        }

        std::cout << std::endl;
        return true;
    }

    bool visit(leaf_node<int> & node, const std::size_t depth) override
    {
        std::cout << "leaf node (" << &node << "): ";
        for (auto i = node.keys_begin(); i != node.keys_end(); ++i)
        {
            std::cout << *i << ' ';
        }
        std::cout << std::endl;
        return true;
    }
};

template <std::size_t O>
bool test_tree()
{
    std::vector<int> keys;
    for (std::size_t i = 0; i < ((O - 1) * 3) + 1; ++i)
    {
        keys.push_back(i);
    }

    std::copy(keys.cbegin(), keys.cend(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;

    std::vector<int> permuted_keys (keys.cbegin(), keys.cend());
    std::sort(permuted_keys.begin(), permuted_keys.end());

    do {

        bplus_tree<int, O> tree;
        for (auto key : permuted_keys)
        {
            // std::cout << key << ' ';

            // if (key == 6)
            // {
            //     std::cout << std::endl << "DEBUG TREE PRINT\n";
            //     printing_visitor printer;
            //     tree.postorder_visit(printer);
            //     std::cout << "DEBUG TREE PRINT\n";
            // }

            tree.insert(key);

            // checking_visitor checker;
            // const bool ret = tree.preorder_visit(checker);

            // if (!ret)
            // {
            //     std::cout << "tree is bad after inserting: " << key << std::endl;
            //     std::copy(permuted_keys.cbegin(),
            //               permuted_keys.cend(),
            //               std::ostream_iterator<int>{std::cout, " "});
            //     std::cout << std::endl;

            //     printing_visitor printer;
            //     tree.postorder_visit(printer);
            //     return false;
            // }
        }

        checking_visitor checker;
        const bool ret = tree.preorder_visit(checker);

        if (!ret)
        {
            std::cout << "tree is bad\n";
            std::copy(permuted_keys.cbegin(),
                      permuted_keys.cend(),
                      std::ostream_iterator<int>{std::cout, " "});
            std::cout << std::endl;
            printing_visitor printer;
            tree.postorder_visit(printer);
            return false;
        }

        if (!std::equal(checker.keys.cbegin(), checker.keys.cend(), keys.cbegin(), keys.cend()))
        {
            std::cout << "keys do not match\n";
            printing_visitor printer;
            tree.postorder_visit(printer);
            return false;
        }

    } while (std::next_permutation(permuted_keys.begin(), permuted_keys.end()));

    return true;
}

int main(int argc, char ** argv)
{
    if (!test_tree<3>())
    {
        std::cout << "tree of order 3 failed\n";
        return 1;
    }

    if (!test_tree<4>())
    {
        std::cout << "tree of order 4 failed\n";
        return 1;
    }

    return 0;
}
