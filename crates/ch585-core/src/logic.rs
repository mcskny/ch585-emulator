#[derive(Debug, Clone, Copy)]
pub struct LogicSample {
    pub cycle: u64,
    pub time_ns: u64,
    pub pa: u16,
    pub pb: u32,
}

/// Change-compressed ring buffer for `PA[15:0]` and `PB[23:0]` pin transitions.
#[derive(Debug, Clone)]
pub struct LogicAnalyzer {
    pub samples: Vec<LogicSample>,
    pub max_samples: usize,
    last_pa: u16,
    last_pb: u32,
}

impl Default for LogicAnalyzer {
    fn default() -> Self {
        Self::new(8192)
    }
}

impl LogicAnalyzer {
    pub fn new(max_samples: usize) -> Self {
        Self {
            samples: Vec::with_capacity(max_samples.min(4096)),
            max_samples: max_samples.max(64),
            last_pa: 0,
            last_pb: 0,
        }
    }

    pub fn clear(&mut self) {
        self.samples.clear();
    }

    #[inline]
    pub fn record_if_changed(&mut self, cycle: u64, time_ns: u64, pa: u16, pb: u32) {
        if self.samples.is_empty() || pa != self.last_pa || pb != self.last_pb {
            self.last_pa = pa;
            self.last_pb = pb;
            if self.samples.len() >= self.max_samples {
                let drop_n = self.max_samples / 8;
                self.samples.drain(0..drop_n);
            }
            self.samples.push(LogicSample {
                cycle,
                time_ns,
                pa,
                pb,
            });
        }
    }
}
