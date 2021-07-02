class TreeNode:
    def __init__(self,left_side=None,right_side=None,value=None):
        self.left_side = left_side
        self.right_side = right_side
        self.val = value
        self.right_child = None
        self.left_child = None
        self.lazy = None

class SegmentTree:
    def __init__(self):
        self.root = None

    def buildTree(self,array):
        self.root = self._buildTree(array,0,len(array))

    def _buildTree(self,array,left,right):
        if left >= right:
            return None
        elif left == right - 1:
            return TreeNode(left,right,array[left])
        else:
            mid = left + (right - left) // 2
            node = TreeNode(left,right)
            node.left_child = self._buildTree(array,left,mid)
            node.right_child = self._buildTree(array,mid,right)
            if node.right_child != None:
                node.val = max(node.left_child.val,node.right_child.val)
            else:
                node.val = node.left_child.val
            return node

    def query(self,left,right):
        return self._query(self.root,left,right)

    def _query(self,node,left,right):
        if left <= node.left_side and right >= node.right_side:
            return node.val
        elif left < node.left_child.left_side and right > node.left_child.right_side:
            return max(self._query(node.left_child,left,right),self._query(node.right_child,left,right))
        elif right <= node.left_child.right_side:
            return self._query(node.left_child,left,right)
        else:
            return self._query(node.right_child, left, right)

    def update(self,index,val):
        self._update(self.root,index,val)

    def _update(self,node,index,val):
        if val > node.val and index >= node.left_side and index < node.right_side :
            node.val = val
            node.lazy = val
            return
        elif val <= node.val:
            if index < node.left_child.right_side:
                self._update(node.left_child,index,val)
            else:
                self._update(node.right_child,index,val)




array = [1,2,3,4,5,200,7,8,9,100]
segmentTree = SegmentTree()
segmentTree.buildTree(array)
print(segmentTree.query(0,10))
segmentTree.update(9,150)
segmentTree.update(7,160)
print(segmentTree.query(7,10))
