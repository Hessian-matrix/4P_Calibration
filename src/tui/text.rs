/// Multiline UTF-8 text buffer shared by the YAML expert editor and the
/// single-line input popup. All cursor positions are byte offsets on character
/// boundaries.
#[derive(Clone, Debug)]
pub struct TextBuffer {
    text: String,
    cursor: usize,
}

impl TextBuffer {
    #[must_use]
    pub fn new(text: String) -> Self {
        Self { text, cursor: 0 }
    }

    #[must_use]
    pub fn text(&self) -> &str {
        &self.text
    }

    #[must_use]
    pub const fn cursor(&self) -> usize {
        self.cursor
    }

    #[must_use]
    pub fn display_text(&self) -> String {
        let mut display = self.text.clone();
        display.insert(self.cursor, '▏');
        display
    }

    #[must_use]
    pub fn cursor_line(&self) -> usize {
        self.text[..self.cursor]
            .bytes()
            .filter(|byte| *byte == b'\n')
            .count()
    }

    pub fn insert_char(&mut self, value: char) {
        self.text.insert(self.cursor, value);
        self.cursor += value.len_utf8();
    }

    pub fn insert_text(&mut self, text: &str) {
        self.text.insert_str(self.cursor, text);
        self.cursor += text.len();
    }

    pub fn backspace(&mut self) {
        if let Some((previous, _)) = self.text[..self.cursor].char_indices().next_back() {
            self.text.drain(previous..self.cursor);
            self.cursor = previous;
        }
    }

    pub fn delete(&mut self) {
        if let Some(value) = self.text[self.cursor..].chars().next() {
            self.text.drain(self.cursor..self.cursor + value.len_utf8());
        }
    }

    pub fn move_left(&mut self) {
        if let Some((previous, _)) = self.text[..self.cursor].char_indices().next_back() {
            self.cursor = previous;
        }
    }

    pub fn move_right(&mut self) {
        if let Some(value) = self.text[self.cursor..].chars().next() {
            self.cursor += value.len_utf8();
        }
    }

    pub fn move_vertical(&mut self, direction: isize) {
        let line_start = self.text[..self.cursor]
            .rfind('\n')
            .map_or(0, |index| index + 1);
        let column = self.text[line_start..self.cursor].chars().count();
        let target_start = if direction < 0 {
            if line_start == 0 {
                return;
            }
            self.text[..line_start - 1]
                .rfind('\n')
                .map_or(0, |index| index + 1)
        } else {
            let Some(next) = self.text[self.cursor..].find('\n') else {
                return;
            };
            self.cursor + next + 1
        };
        let target_end = self.text[target_start..]
            .find('\n')
            .map_or(self.text.len(), |index| target_start + index);
        self.cursor = self.text[target_start..target_end]
            .char_indices()
            .nth(column)
            .map_or(target_end, |(index, _)| target_start + index);
    }

    pub fn move_home(&mut self) {
        self.cursor = self.text[..self.cursor]
            .rfind('\n')
            .map_or(0, |index| index + 1);
    }

    pub fn move_end(&mut self) {
        self.cursor = self.text[self.cursor..]
            .find('\n')
            .map_or(self.text.len(), |index| self.cursor + index);
    }
}

#[cfg(test)]
mod tests {
    use super::TextBuffer;

    #[test]
    fn plain_letters_are_text_not_shortcuts() {
        let mut buffer = TextBuffer::new(String::new());
        for character in "imu/rmqe/calibration.yaml".chars() {
            buffer.insert_char(character);
        }
        assert_eq!(buffer.text(), "imu/rmqe/calibration.yaml");
    }

    #[test]
    fn editing_respects_utf8_boundaries() {
        let mut buffer = TextBuffer::new("相机".to_owned());
        buffer.insert_char('A');
        assert_eq!(buffer.text(), "A相机");
        buffer.move_end();
        buffer.insert_char('B');
        assert_eq!(buffer.text(), "A相机B");
        buffer.backspace();
        assert_eq!(buffer.text(), "A相机");
        buffer.move_home();
        buffer.delete();
        assert_eq!(buffer.text(), "相机");
    }
}
