// AFC prompts for the web clients' action:prompt dialogs (port of
// AFC_respond.py)
//
// Copyright (C) 2024-2026 Armored Turtle
//
// This file may be distributed under the terms of the GNU GPLv3 license.
use super::codes::*;
use super::logger::AfcLogger;

/// A button: (label, command, style)
pub type Button = (String, String, Option<String>);

pub struct AfcPrompt<'a> {
    logger: &'a AfcLogger,
}

impl<'a> AfcPrompt<'a> {
    pub fn new(logger: &'a AfcLogger) -> Self {
        AfcPrompt { logger }
    }

    pub fn p_begin(&self, prompt_name: &str) {
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_BEGIN, &format!("// action:prompt_begin {}", prompt_name));
    }

    pub fn p_text(&self, text: &str) {
        for line in text.split('\n') {
            self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_TEXT, &format!("// action:prompt_text {}", line));
        }
    }

    pub fn p_button(&self, label: &str, command: &str, style: Option<&str>) {
        let msg = match style.filter(|s| !s.is_empty()) {
            Some(style) => format!("// action:prompt_button {}|{}|{}", label, command, style),
            None => format!("// action:prompt_button {}|{}", label, command),
        };
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_BUTTON, &msg);
    }

    pub fn p_footer_button(&self, label: &str, command: &str, style: Option<&str>) {
        let msg = match style.filter(|s| !s.is_empty()) {
            Some(style) => format!("// action:prompt_footer_button {}|{}|{}", label, command, style),
            None => format!("// action:prompt_footer_button {}|{}", label, command),
        };
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_FOOTER_BUTTON, &msg);
    }

    pub fn p_cancel_button(&self) {
        self.p_footer_button("Cancel", "RESPOND TYPE=command MSG=action:prompt_end", Some("warning"));
    }

    pub fn p_show(&self) {
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_SHOW, "// action:prompt_show");
    }

    pub fn p_end(&self) {
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_END, "// action:prompt_end");
    }

    pub fn p_button_group_start(&self) {
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_BUTTON_GROUP_START, "// action:prompt_button_group_start");
    }

    pub fn p_button_group_end(&self) {
        self.logger.raw(&AFC_RESPOND_AFCPROMPT_P_BUTTON_GROUP_END, "// action:prompt_button_group_end");
    }

    pub fn create_custom_p(
        &self,
        prompt_name: &str,
        text: Option<&str>,
        buttons: Option<&[Button]>,
        cancel: bool,
        groups: Option<&[Vec<Button>]>,
        footer_buttons: Option<&[Button]>,
    ) {
        self.p_begin(prompt_name);
        if let Some(text) = text {
            self.p_text(text);
        }
        for group in groups.unwrap_or_default() {
            self.p_button_group_start();
            for (label, command, style) in group {
                self.p_button(label, command, style.as_deref());
            }
            self.p_button_group_end();
        }
        for (label, command, style) in buttons.unwrap_or_default() {
            self.p_button(label, command, style.as_deref());
        }
        for (label, command, style) in footer_buttons.unwrap_or_default() {
            self.p_footer_button(label, command, style.as_deref());
        }
        if cancel {
            self.p_cancel_button();
        }
        self.p_show();
    }
}

/// (label, command, style) as python's tuples
pub fn button(label: impl Into<String>, command: impl Into<String>, style: &str) -> Button {
    (label.into(), command.into(), Some(style.to_string()))
}
