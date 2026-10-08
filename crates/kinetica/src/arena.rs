//! Generational arena storing bodies and colliders behind stable handles.
//!
//! Removing a value frees its slot for reuse and bumps the slot generation, so a handle to a
//! removed value never silently refers to whatever is stored in that slot afterwards.

/// Position of a value in an [`Arena`], valid only for the generation it was created with.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) struct Index {
    slot: u32,
    generation: u32,
}

#[derive(Debug, Clone)]
enum Entry<T> {
    Occupied { generation: u32, value: T },
    Free { generation: u32, next_free: Option<u32> },
}

/// A vector of slots where removed values leave reusable holes.
///
/// Iteration follows slot order, which only depends on the sequence of insertions and removals,
/// so it is deterministic.
#[derive(Debug, Clone)]
pub(crate) struct Arena<T> {
    entries: Vec<Entry<T>>,
    first_free: Option<u32>,
    len: usize,
}

impl<T> Default for Arena<T> {
    fn default() -> Self {
        Self { entries: Vec::new(), first_free: None, len: 0 }
    }
}

impl<T> Arena<T> {
    pub(crate) fn insert(&mut self, value: T) -> Index {
        self.len += 1;
        if let Some(slot) = self.first_free {
            let entry = &mut self.entries[slot as usize];
            let Entry::Free { generation, next_free } = *entry else {
                unreachable!("the free list only links free entries");
            };
            self.first_free = next_free;
            *entry = Entry::Occupied { generation, value };
            Index { slot, generation }
        } else {
            let slot = u32::try_from(self.entries.len()).expect("arena holds more than u32::MAX values");
            self.entries.push(Entry::Occupied { generation: 0, value });
            Index { slot, generation: 0 }
        }
    }

    pub(crate) fn remove(&mut self, index: Index) -> Option<T> {
        self.get(index)?;
        let freed = Entry::Free { generation: index.generation.wrapping_add(1), next_free: self.first_free };
        let Entry::Occupied { value, .. } = std::mem::replace(&mut self.entries[index.slot as usize], freed) else {
            unreachable!("`get` just checked that the entry is occupied");
        };
        self.first_free = Some(index.slot);
        self.len -= 1;
        Some(value)
    }

    pub(crate) fn get(&self, index: Index) -> Option<&T> {
        match self.entries.get(index.slot as usize)? {
            Entry::Occupied { generation, value } if *generation == index.generation => Some(value),
            _ => None,
        }
    }

    pub(crate) fn get_mut(&mut self, index: Index) -> Option<&mut T> {
        match self.entries.get_mut(index.slot as usize)? {
            Entry::Occupied { generation, value } if *generation == index.generation => Some(value),
            _ => None,
        }
    }

    pub(crate) fn len(&self) -> usize {
        self.len
    }

    pub(crate) fn iter(&self) -> impl Iterator<Item = (Index, &T)> {
        self.entries.iter().zip(0..).filter_map(|(entry, slot)| match entry {
            Entry::Occupied { generation, value } => Some((Index { slot, generation: *generation }, value)),
            Entry::Free { .. } => None,
        })
    }

    pub(crate) fn values_mut(&mut self) -> impl Iterator<Item = &mut T> {
        self.entries.iter_mut().filter_map(|entry| match entry {
            Entry::Occupied { value, .. } => Some(value),
            Entry::Free { .. } => None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn insert_then_get() {
        let mut arena = Arena::default();
        let a = arena.insert("a");
        let b = arena.insert("b");
        assert_eq!(arena.get(a), Some(&"a"));
        assert_eq!(arena.get(b), Some(&"b"));
        assert_eq!(arena.len(), 2);
    }

    #[test]
    fn removed_index_is_invalid_even_after_slot_reuse() {
        let mut arena = Arena::default();
        let old = arena.insert(1);
        assert_eq!(arena.remove(old), Some(1));
        assert_eq!(arena.remove(old), None);

        let new = arena.insert(2);
        assert_eq!(new.slot, old.slot, "the freed slot should be reused");
        assert_eq!(arena.get(old), None);
        assert_eq!(arena.get(new), Some(&2));
    }

    #[test]
    fn iteration_skips_free_slots() {
        let mut arena = Arena::default();
        let indices: Vec<_> = (0..4).map(|i| arena.insert(i)).collect();
        arena.remove(indices[1]);
        let values: Vec<_> = arena.iter().map(|(_, v)| *v).collect();
        assert_eq!(values, [0, 2, 3]);
    }
}
