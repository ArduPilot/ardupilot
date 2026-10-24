// SPDX-License-Identifier: GPL-3.0-or-later

use crate::{Context, Location, Result, Vector3f};

impl Location {
    #[deprecated(note = "use get_vector_from_origin_neu_cm or get_vector_from_origin_neu_m")]
    pub fn get_vector_from_origin_neu(&mut self, context: &Context<'_>) -> Result<Option<Vector3f>> {
        self.get_vector_from_origin_neu_cm(context)
    }
}
